#!/usr/bin/env python3
"""Drive /cmd_vel from the Isaac-trained policy.

WHAT THIS IS
    The deployment half of research/isaac. It loads the ONNX policy exported
    by research/isaac/scripts/play.py --export, feeds it the same observation
    the Isaac environment fed it, and publishes geometry_msgs/Twist on
    /cmd_vel at 8 Hz -- nav2_params.yaml's controller_frequency.

    It replaces Nav2's controller chain, so DO NOT run navigation_launch.py
    alongside it: nav2's collision_monitor is the final /cmd_vel publisher
    there and the two will fight. Lane detection must be running, because the
    policy eats /scan_fused and nothing else publishes it.

THE OBSERVATION IS NOT REBUILT HERE
    Every number comes from research/isaac/obs_core.py, the same module the
    trainer imports. Beam pooling, goal encoding, action scaling and the
    acceleration limits are defined once. A reimplementation would agree right
    up until one side was edited, and the symptom -- fine in Isaac, wandering
    on the robot -- looks nothing like its cause.

    policy_meta.yaml is checked against that module before the first command
    goes out. A policy trained with 128 pooled beams, loaded by a node that
    pools to 96, would otherwise produce confident, wrong velocities instead
    of an error.

ONE SUBTLETY WORTH THE WORDS
    The velocity fed into the observation is the COMMANDED twist this node
    last sent, not the measured one from odometry. That is what the training
    environment used, and the two differ on a real robot: wheels slip, the
    controller lags. Feeding measured velocity here would be a silent
    train/deploy mismatch. Odometry is used for the robot's POSE, which is a
    different question.

GOALS
    Training steered the policy toward a point about 4 m ahead on the course
    centreline. On the robot there is no centreline, so:
      - a PoseStamped on ~goal_topic (RViz's "2D Goal Pose") is used if fresh
      - otherwise a carrot is projected ~goal_lookahead metres straight ahead,
        which makes the policy drive forward and follow the lane it can see

SAFETY
    Publishing stops and a zero Twist goes out if the scan or odometry goes
    stale, and ~enable_cmd_vel can be set false to run the node silently while
    watching its debug output -- the same convention track_follower.py and
    follow_the_gap.py use.

RUN IT
    ros2 run esda_simulation_2025 rl_policy_node.py --ros-args \\
        -p policy_path:=<...>/exported/policy.onnx
"""
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

# --------------------------------------------------------------- obs_core --
# Not installed as a package: research/isaac is offline research code that the
# ROS build does not touch. Look in the places it can actually be, and say so
# clearly rather than dying on an ImportError.
_CANDIDATES = [
    os.environ.get("ESDA_ISAAC_DIR", ""),
    os.path.join(os.getcwd(), "research", "isaac"),
    os.path.abspath(os.path.join(os.path.dirname(__file__),
                                 "..", "..", "..", "research", "isaac")),
]
for _path in _CANDIDATES:
    if _path and os.path.exists(os.path.join(_path, "obs_core.py")):
        sys.path.insert(0, _path)
        break
try:
    import obs_core as OC
except ImportError:
    sys.exit(
        "cannot import obs_core.py. It lives in research/isaac/ and is the\n"
        "shared definition of what the policy sees. Either run from the\n"
        "workspace root, or set ESDA_ISAAC_DIR to that directory.\n"
        "Looked in:\n  " + "\n  ".join(p for p in _CANDIDATES if p))


LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)


class RLPolicyNode(Node):
    def __init__(self):
        super().__init__("rl_policy_node")

        self.declare_parameter("policy_path", "")
        self.declare_parameter("meta_path", "")
        self.declare_parameter("scan_topic", OC.policy_meta()["scan_topic"])
        self.declare_parameter("odom_topic", OC.policy_meta()["odom_topic"])
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("cmd_vel_topic", OC.policy_meta()["cmd_vel_topic"])
        self.declare_parameter("control_hz", OC.CONTROL_HZ)
        self.declare_parameter("goal_lookahead", 4.0)
        self.declare_parameter("goal_timeout", 5.0)
        self.declare_parameter("input_timeout", 0.5)
        self.declare_parameter("enable_cmd_vel", True)
        self.declare_parameter("debug", False)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.control_hz = float(g("control_hz"))
        self.goal_lookahead = float(g("goal_lookahead"))
        self.goal_timeout = float(g("goal_timeout"))
        self.input_timeout = float(g("input_timeout"))
        self.enable_cmd_vel = bool(g("enable_cmd_vel"))
        self.debug = bool(g("debug"))

        self._load_policy(str(g("policy_path")), str(g("meta_path")))

        self.scan = None
        self.scan_stamp = None
        self.scan_warned = False
        self.pose = None            # (x, y, yaw) from odometry
        self.odom_stamp = None
        self.goal = None            # (x, y)
        self.goal_stamp = None

        # The commanded twist, NOT the measured one -- see the module docstring.
        self.twist = np.zeros(2)
        self.prev_action = np.zeros(OC.ACTION_DIM)
        self.frames = [np.zeros(OC.OBS_PER_FRAME) for _ in range(OC.N_HISTORY)]

        self.create_subscription(LaserScan, str(g("scan_topic")), self._on_scan, 10)
        self.create_subscription(Odometry, str(g("odom_topic")), self._on_odom, 10)
        self.create_subscription(PoseStamped, str(g("goal_topic")), self._on_goal, 10)
        self.pub = self.create_publisher(Twist, str(g("cmd_vel_topic")), 10)
        self.create_timer(1.0 / self.control_hz, self._tick)

        self.get_logger().info(
            "rl_policy_node ready: %d-beam scan -> %d pooled x %d frames = %d obs, "
            "publishing %s at %.1f Hz%s"
            % (OC.N_BEAMS_RAW, OC.N_POOLED, OC.N_HISTORY, OC.OBS_DIM,
               g("cmd_vel_topic"), self.control_hz,
               "" if self.enable_cmd_vel else "  [enable_cmd_vel FALSE -- not publishing]"))
        self.get_logger().warn(
            "do not run navigation_launch.py at the same time: nav2's "
            "collision_monitor also publishes /cmd_vel")

    # ------------------------------------------------------------- policy --
    def _load_policy(self, policy_path, meta_path):
        if not policy_path:
            self.get_logger().fatal("set the policy_path parameter to a policy.onnx")
            raise SystemExit(1)
        if not os.path.exists(policy_path):
            self.get_logger().fatal("no such policy: %s" % policy_path)
            raise SystemExit(1)

        meta_path = meta_path or os.path.join(os.path.dirname(policy_path),
                                              "policy_meta.yaml")
        if os.path.exists(meta_path):
            import yaml
            with open(meta_path) as fh:
                loaded = yaml.safe_load(fh) or {}
            problems = OC.check_meta(loaded)
            if problems:
                self.get_logger().fatal(
                    "this policy was trained against a different contract:\n  "
                    + "\n  ".join(problems))
                raise SystemExit(1)
            self.get_logger().info("policy_meta.yaml agrees with obs_core")
        else:
            self.get_logger().warn(
                "no policy_meta.yaml beside the policy (%s). Running anyway, "
                "but nothing is checking that this policy was trained against "
                "the current obs_core." % meta_path)

        try:
            import onnxruntime as ort
        except ImportError:
            self.get_logger().fatal(
                "onnxruntime is not installed:  "
                "pip3 install onnxruntime==1.18.1  -- "
                "1.18.1 specifically: it is the only release in range whose "
                "own metadata caps numpy<2.0, which is what cv_bridge needs. "
                "1.19 and later leave numpy uncapped, so pip is free to pull "
                "numpy 2.x and break cv_bridge. All of them need numpy>=1.21.6, "
                "so expect a small numpy bump from the 1.21.5 Humble ships.")
            raise SystemExit(1)

        self.session = ort.InferenceSession(policy_path,
                                            providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        shape = self.session.get_inputs()[0].shape
        width = shape[-1] if shape else None
        if isinstance(width, int) and width != OC.OBS_DIM:
            self.get_logger().fatal(
                "policy expects %d inputs, obs_core builds %d" % (width, OC.OBS_DIM))
            raise SystemExit(1)
        self.get_logger().info("loaded %s (input %s)" % (policy_path, shape))

    # --------------------------------------------------------- callbacks --
    def _on_scan(self, msg):
        self.scan = np.asarray(msg.ranges, dtype=np.float64)
        self.scan_stamp = self.get_clock().now()

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.odom_stamp = self.get_clock().now()

    def _on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_stamp = self.get_clock().now()
        self.get_logger().info("goal set: (%.2f, %.2f)" % self.goal)

    # -------------------------------------------------------------- tick --
    def _age(self, stamp):
        if stamp is None:
            return float("inf")
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _pool(self, ranges):
        """898 beams -> N_POOLED, through obs_core so training and here agree."""
        if ranges.shape[0] == OC.N_BEAMS_RAW:
            return OC.pool_scan(ranges)
        if not self.scan_warned:
            self.scan_warned = True
            self.get_logger().warn(
                "scan has %d beams, not the %d the policy was trained on. "
                "Resampling, but the mapping from beam to bucket is no longer "
                "the one it learned." % (ranges.shape[0], OC.N_BEAMS_RAW))
        return OC.resample_scan(ranges, OC.N_POOLED)

    def _goal_point(self, x, y, yaw):
        """The goal, or a carrot straight ahead when none is fresh."""
        if self.goal is not None and self._age(self.goal_stamp) < self.goal_timeout:
            return self.goal, True
        return (x + self.goal_lookahead * math.cos(yaw),
                y + self.goal_lookahead * math.sin(yaw)), False

    def _stop(self, reason):
        self.twist[:] = 0.0
        if self.enable_cmd_vel:
            self.pub.publish(Twist())
        if reason:
            self.get_logger().warn(reason, throttle_duration_sec=2.0)

    def _tick(self):
        scan_age, odom_age = self._age(self.scan_stamp), self._age(self.odom_stamp)
        if scan_age > self.input_timeout:
            return self._stop("no %s for %.1f s -- is lane detection running?"
                              % (self.get_parameter("scan_topic").value, scan_age))
        if odom_age > self.input_timeout:
            return self._stop("no odometry for %.1f s" % odom_age)

        x, y, yaw = self.pose
        goal, have_goal = self._goal_point(x, y, yaw)

        frame = OC.encode_frame(self._pool(self.scan), self.twist, (x, y), yaw,
                                goal, self.prev_action)
        self.frames.pop(0)
        self.frames.append(frame)
        obs = OC.stack_history(self.frames).astype(np.float32)[None, :]

        action = np.asarray(self.session.run(None, {self.input_name: obs})[0],
                            dtype=np.float64).ravel()[:OC.ACTION_DIM]

        # Rate limited exactly as diff_drive_controller limits it, and exactly
        # as the training environment did.
        self.twist = OC.action_to_twist(action, self.twist, dt=1.0 / self.control_hz)
        self.prev_action = np.clip(action, -1.0, 1.0)

        msg = Twist()
        msg.linear.x = float(self.twist[0])
        msg.angular.z = float(self.twist[1])
        if self.enable_cmd_vel:
            self.pub.publish(msg)

        if self.debug:
            pooled = OC.pool_scan(self.scan) if self.scan.shape[0] == OC.N_BEAMS_RAW \
                else OC.resample_scan(self.scan, OC.N_POOLED)
            self.get_logger().info(
                "v %+.2f w %+.2f | nearest %.2f m | goal %s (%.1f m)%s"
                % (self.twist[0], self.twist[1], float(pooled.min()),
                   "set" if have_goal else "carrot",
                   math.hypot(goal[0] - x, goal[1] - y),
                   "" if self.enable_cmd_vel else "  [not published]"),
                throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = RLPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the robot stopped, not coasting on the last command.
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
