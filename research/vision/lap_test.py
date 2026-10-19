#!/usr/bin/env python3
"""
lap_test.py -- did the robot actually get round the course?

Turns "it drove for a bit then stopped" into numbers that can be compared
between runs and between worlds. Writes a JSON report plus a human summary
into <repo>/logs/, where they survive the run and can be read by someone who
was not watching.

WHY THE MEASUREMENT HAS TO BE DELIBERATE

Eyeballing the Gazebo window rewards whatever the parameters were last tuned
against. The honest question is whether the robot completes a lap of a course
it has never been tuned on, so this records a held-out world by name and
refuses to pretend a partial run was a lap:

    lap           left a circle of --depart m around the start, turned
                  through at least --min-turn rad net, travelled at least
                  --min-lap m, and came back within --close m of the start
    progress      furthest it ever got from the start, and total path length
    stalls        periods below --stall-speed; the usual failure is the lane
                  detector going blind, and this counts how much of the run
                  was spent not moving
    clearance     closest lidar return during the whole run -- a lap that
                  clipped a barrel is not a clean lap

THE TURN TERM MATTERS. Distance-and-return alone scores an out-and-back as a
lap: drive 20 m up the course, turn round, drive 20 m home, and you have left
the circle, covered 40 m and returned -- without ever going round anything.
The robot has in fact done this. What separates the two is NET heading change,
which by the theorem of turning tangents is +-2*pi for any simple closed loop
however wiggly, and only +-pi for an out-and-back (the one U-turn at the far
end). Weaving between barrels adds and subtracts and cancels out, so it does
not inflate the figure.

Note the angle is measured on the robot's own heading, NOT on its bearing from
the start: for a start point sitting on the track, one lap sweeps only pi of
bearing, so that test would reject every real lap.

The turn is counted per lap segment, not from the beginning of the run, so a
figure of eight entered at its crossing scores each lobe separately: +2*pi,
then -2*pi, two laps. Entered anywhere else, one circuit of an eight nets 0
and is rejected -- so run igvc_figure8* with --min-turn 0 and lean on the
distance and return terms there.

Time comes from the odometry header, not the wall clock, so a run on a laptop
at 0.4x real time is still measured in simulated seconds and is comparable
with a run on a faster machine.

USAGE
    python3 lap_test.py --world igvc_oval_test --seconds 300
    python3 lap_test.py --world igvc_oval_test --laps 2      # stop after 2
    python3 test_lap_test.py                                 # offline tests
"""
import argparse
import json
import math
import os
import time

# ROS is only needed to *run* the test; the scoring logic below is plain
# Python so it can be exercised offline by test_lap_test.py.
try:
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, PointCloud2
    from geometry_msgs.msg import PoseStamped
    HAVE_ROS = True
except ImportError:                                   # offline test run
    HAVE_ROS = False
    Node = object


# --------------------------------------------------------------------------
# scoring -- no ROS, no I/O, fully testable
# --------------------------------------------------------------------------
class LapTracker:
    """Accumulates pose samples and decides what counts as a lap.

    Feed it update(t, x, y, v) in time order. t is seconds (simulated), v is
    speed in m/s; pass v=None to have it differentiated from the positions.
    """

    def __init__(self, depart=6.0, close=4.0, min_lap=30.0, min_turn=5.5,
                 stall_speed=0.05, stall_min=1.0, track_every=1.0):
        if close >= depart:
            raise ValueError("--close must be smaller than --depart, else the "
                             "robot is 'returned' before it has ever left")
        self.depart, self.close = depart, close
        self.min_lap, self.min_turn = min_lap, min_turn
        self.stall_speed, self.stall_min = stall_speed, stall_min
        self.track_every = track_every

        self.p0 = None          # start position
        self.prev = None        # previous position
        self.t0 = None
        self.t = None
        self.path_len = 0.0
        self.max_dist = 0.0
        self.departed = False
        self.laps = []          # (index, elapsed_s, lap_length_m, lap_turn_rad)
        self.lap_base_len = 0.0
        self.lap_base_turn = 0.0
        self.turn_net = 0.0     # signed heading change, unwrapped: +-2pi a lap
        self.turn_abs = 0.0     # unsigned, for the report only
        self._last_yaw = None
        self.stalls = []        # (start_elapsed_s, duration_s)
        self._stall_from = None
        self.samples = 0
        self.track = []         # [(elapsed_s, x, y)]

    # ...................................................................
    def update(self, t, x, y, yaw=None, v=None):
        """One pose sample. Returns the lap tuple if a lap just closed.

        yaw is the robot's heading in rad; pass None to have it taken from the
        direction of travel instead (noisy below ~0.1 m/s, so prefer the real
        one when odometry supplies it).
        """
        if self.p0 is None:
            self.p0, self.prev, self.t0, self.t = (x, y), (x, y), t, t
            self.track.append((0.0, round(x, 2), round(y, 2)))
            return None
        if t < self.t:                       # clock jumped back (sim reset)
            return None
        dt = t - self.t
        self.t = t
        self.samples += 1

        px, py = self.prev
        step = math.hypot(x - px, y - py)
        self.path_len += step
        self.prev = (x, y)

        speed = v if v is not None else (step / dt if dt > 1e-6 else 0.0)
        self._stall(t, speed)

        dx, dy = x - self.p0[0], y - self.p0[1]
        d0 = math.hypot(dx, dy)
        self.max_dist = max(self.max_dist, d0)

        # net heading change, unwrapped. Derived from the track only if the
        # robot actually moved this step, else the direction is meaningless.
        if yaw is None and step > 0.02:
            yaw = math.atan2(y - py, x - px)
        if yaw is not None:
            if self._last_yaw is not None:
                dyaw = (yaw - self._last_yaw + math.pi) % (2 * math.pi) - math.pi
                self.turn_net += dyaw
                self.turn_abs += abs(dyaw)
            self._last_yaw = yaw

        if d0 > self.depart:
            self.departed = True
        lap = None
        if (self.departed and d0 < self.close
                and (self.path_len - self.lap_base_len) >= self.min_lap
                and abs(self.turn_net - self.lap_base_turn) >= self.min_turn):
            lap = (len(self.laps) + 1,
                   round(t - self.t0, 1),
                   round(self.path_len - self.lap_base_len, 1),
                   round(self.turn_net - self.lap_base_turn, 2))
            self.laps.append(lap)
            self.lap_base_len = self.path_len
            self.lap_base_turn = self.turn_net
            self.departed = False

        if (t - self.t0) - self.track[-1][0] >= self.track_every:
            self.track.append((round(t - self.t0, 1), round(x, 2), round(y, 2)))
        return lap

    # ...................................................................
    def _stall(self, t, speed):
        if speed < self.stall_speed:
            if self._stall_from is None:
                self._stall_from = t
        elif self._stall_from is not None:
            d = t - self._stall_from
            if d >= self.stall_min:
                self.stalls.append((round(self._stall_from - self.t0, 1),
                                    round(d, 1)))
            self._stall_from = None

    def finish(self):
        """Close an open stall so the last one is not silently dropped."""
        if self._stall_from is not None and self.t is not None:
            d = self.t - self._stall_from
            if d >= self.stall_min:
                self.stalls.append((round(self._stall_from - self.t0, 1),
                                    round(d, 1)))
            self._stall_from = None

    # ...................................................................
    @property
    def elapsed(self):
        return 0.0 if self.t0 is None else self.t - self.t0

    @property
    def stalled_s(self):
        return sum(d for _, d in self.stalls)

    @property
    def dist_from_start(self):
        if self.p0 is None:
            return 0.0
        return math.hypot(self.prev[0] - self.p0[0], self.prev[1] - self.p0[1])


# --------------------------------------------------------------------------
# ROS wrapper
# --------------------------------------------------------------------------
def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LapTest(Node):
    def __init__(self, a):
        super().__init__("lap_test")
        self.a = a
        self.lt = LapTracker(depart=a.depart, close=a.close, min_lap=a.min_lap,
                             min_turn=a.min_turn, stall_speed=a.stall_speed,
                             stall_min=a.stall_min)
        self.min_clearance = float("inf")
        # Subscribe to BOTH candidate odometry topics and lock onto whichever
        # speaks first. ros2_control_2wd.xacro remaps the controller's odom to
        # plain /odom, and defaulting to the un-remapped name cost a whole
        # 420 s benchmark run that logged "no odom yet" 84 times and produced
        # an empty report. A measurement tool that silently measures nothing
        # is the worst kind.
        self._odom_src = None
        for t in (a.odom_topic, a.odom_topic_alt):
            if t:
                self.create_subscription(
                    Odometry, t, (lambda m, _t=t: self._on_odom(m, _t)), 20)
        self.create_subscription(LaserScan, a.scan_topic,
                                 self._on_scan, qos_profile_sensor_data)

        # Perception throughput, counted here so the report is self-contained.
        #
        # In three scored runs these two numbers and the distance travelled
        # moved together exactly -- 42 lane publishes, 42 goals, 33 m; then
        # 20, 20, 13 m; then 22, 22, 21 m. The robot goes as far as it is
        # told to, and it is told once per camera frame that gets through. A
        # report that gives only metres hides the one quantity that explains
        # them, which is why every comparison from here on quotes carrots per
        # simulated second alongside the distance.
        self.n_lane = 0
        self.n_goal = 0
        self.create_subscription(PointCloud2, a.lane_topic,
                                 self._on_lane, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, a.goal_topic, self._on_goal, 10)

        self.create_timer(5.0, self._progress)
        self.get_logger().info(
            f"lap_test on '{a.world}': depart>{a.depart} m, close<{a.close} m, "
            f"lap >= {a.min_lap} m and {a.min_turn} rad, limit {a.seconds}s wall")

    def _on_lane(self, _msg):
        self.n_lane += 1

    def _on_goal(self, _msg):
        self.n_goal += 1

    def _on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        if ok.any():
            self.min_clearance = min(self.min_clearance, float(r[ok].min()))

    def _on_odom(self, msg, topic="?"):
        if self._odom_src is None:
            self._odom_src = topic
            self.get_logger().info(f"odometry arriving on {topic}")
        elif topic != self._odom_src:
            return                      # never mix two sources mid-run
        p = msg.pose.pose.position
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        lap = self.lt.update(stamp_s(msg), p.x, p.y,
                             yaw_of(msg.pose.pose.orientation), v)
        if lap:
            self.get_logger().info(
                f"*** LAP {lap[0]} complete: {lap[2]} m, {lap[3]} rad swept, "
                f"at t={lap[1]} s ***")

    def _progress(self):
        lt = self.lt
        if lt.t0 is None:
            self.get_logger().warn(
                f"no odometry yet on {self.a.odom_topic} "
                f"or {self.a.odom_topic_alt}")
            return
        self.get_logger().info(
            f"t={lt.elapsed:6.0f}s  path={lt.path_len:6.1f} m  "
            f"from start={lt.dist_from_start:5.1f} m  max={lt.max_dist:5.1f} m  "
            f"turn={lt.turn_net:+5.1f} rad  laps={len(lt.laps)}  "
            f"stalls={len(lt.stalls)}")

    def report(self):
        lt = self.lt
        lt.finish()
        el = lt.elapsed
        r = {
            "world": self.a.world,
            "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
            "held_out": self.a.world.endswith("_test"),
            "note": self.a.note,
            "duration_s": round(el, 1),
            "path_length_m": round(lt.path_len, 1),
            "laps_completed": len(lt.laps),
            "laps": lt.laps,
            "max_distance_from_start_m": round(lt.max_dist, 1),
            "turn_net_rad": round(lt.turn_net, 2),
            "turn_abs_rad": round(lt.turn_abs, 2),
            "mean_speed_mps": round(lt.path_len / el, 2) if el > 1 else 0.0,
            "stall_count": len(lt.stalls),
            "stalled_s": round(lt.stalled_s, 1),
            "stalled_fraction": round(lt.stalled_s / el, 3) if el > 1 else 0.0,
            "min_lidar_clearance_m": (None if self.min_clearance == float("inf")
                                      else round(self.min_clearance, 2)),
            "odom_samples": lt.samples,
            "odom_topic_used": self._odom_src,
            "lane_publishes": self.n_lane,
            "goals_published": self.n_goal,
            "lane_per_sim_s": round(self.n_lane / el, 3) if el > 1 else 0.0,
            "goals_per_sim_s": round(self.n_goal / el, 3) if el > 1 else 0.0,
            "metres_per_goal": (round(lt.path_len / self.n_goal, 2)
                                if self.n_goal else None),
            "track": lt.track,
            "params": {"depart": self.a.depart, "close": self.a.close,
                       "min_lap": self.a.min_lap, "min_turn": self.a.min_turn,
                       "stall_speed": self.a.stall_speed,
                       "stall_min": self.a.stall_min},
        }
        return r


def format_report(r):
    verdict = ("PASS - completed %d lap(s)" % r["laps_completed"]
               if r["laps_completed"] else "FAIL - no lap completed")
    bias = ("   [HELD OUT]" if r["held_out"]
            else "   [** tuned-on world, result is biased **]")
    return f"""LAP TEST  {r['world']}{bias}
{r['generated']}{('  -- ' + r['note']) if r.get('note') else ''}

  {verdict}

  duration            {r['duration_s']} s (simulated)
  path length         {r['path_length_m']} m
  mean speed          {r['mean_speed_mps']} m/s
  furthest from start {r['max_distance_from_start_m']} m
  net heading change  {r['turn_net_rad']} rad  (a lap is +-6.28)
  total turning       {r['turn_abs_rad']} rad
  laps                {r['laps']}
  stalls              {r['stall_count']}  totalling {r['stalled_s']} s ({r['stalled_fraction']*100:.0f}% of the run)
  min lidar clearance {r['min_lidar_clearance_m']} m
  odom samples        {r['odom_samples']}  from {r.get('odom_topic_used')}

  lane publishes      {r.get('lane_publishes', '?')}  ({r.get('lane_per_sim_s', '?')} per simulated second)
  goals published     {r.get('goals_published', '?')}  ({r.get('goals_per_sim_s', '?')} per simulated second)
  metres per goal     {r.get('metres_per_goal')}

  lap rule: leave a {r['params']['depart']} m circle, travel >= {r['params']['min_lap']} m,
            turn through >= {r['params']['min_turn']} rad net, return within
            {r['params']['close']} m of the start.
"""


def write_report(r, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    stem = "lap_%s_%s" % (r["world"], time.strftime("%Y%m%d_%H%M%S"))
    with open(os.path.join(out_dir, stem + ".json"), "w") as f:
        json.dump(r, f, indent=2)
    txt = format_report(r)
    with open(os.path.join(out_dir, stem + ".txt"), "w") as f:
        f.write(txt)
    with open(os.path.join(out_dir, "lap_latest.txt"), "w") as f:
        f.write(txt)
    print(txt)
    return os.path.join(out_dir, stem + ".txt")


def build_parser():
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", default="unknown")
    p.add_argument("--note", default="", help="free text stored in the report")
    p.add_argument("--odom-topic", default="/odom",
                   help="ros2_control_2wd.xacro remaps the controller's "
                        "odometry to this")
    p.add_argument("--odom-topic-alt", default="/diff_drive_base_controller/odom",
                   help="also subscribed; whichever topic speaks first is "
                        "used for the whole run")
    p.add_argument("--scan-topic", default="/scan")
    p.add_argument("--lane-topic", default="/lane/points",
                   help="counted, not read: one publish is one chance to steer")
    p.add_argument("--goal-topic", default="/goal_pose",
                   help="counted, not read: one carrot the driver sent to Nav2")
    p.add_argument("--out", default=os.path.join(here, "..", "logs"))
    p.add_argument("--seconds", type=float, default=300.0,
                   help="wall-clock time limit")
    p.add_argument("--laps", type=int, default=0, help="stop after N laps (0 = no limit)")
    p.add_argument("--depart", type=float, default=6.0,
                   help="must get this far from the start before a return counts")
    p.add_argument("--close", type=float, default=4.0,
                   help="returning within this of the start closes a lap")
    p.add_argument("--min-lap", type=float, default=30.0,
                   help="minimum path length for a lap, so a wobble near the "
                        "start cannot register as one")
    p.add_argument("--min-turn", type=float, default=5.5,
                   help="minimum |net heading change| over a lap, rad; stops "
                        "an out-and-back (which nets only pi) scoring as a "
                        "lap. Use 0 for figure-of-eight worlds, whose two "
                        "loops cancel")
    p.add_argument("--stall-speed", type=float, default=0.05)
    p.add_argument("--stall-min", type=float, default=1.0,
                   help="ignore pauses shorter than this, s")
    return p


def main():
    a = build_parser().parse_args()
    if not HAVE_ROS:
        raise SystemExit("rclpy not importable -- source the ROS workspace first")
    rclpy.init()
    n = LapTest(a)
    t_end = time.time() + a.seconds
    try:
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.2)
            if a.laps and len(n.lt.laps) >= a.laps:
                n.get_logger().info("lap target reached")
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            write_report(n.report(), os.path.abspath(a.out))
        finally:
            n.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
