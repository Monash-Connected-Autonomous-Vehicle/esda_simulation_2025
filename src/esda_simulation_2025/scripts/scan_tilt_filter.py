#!/usr/bin/env python3
"""
scan_tilt_filter.py -- drop lidar returns that are the GROUND seen by a
tilted robot.

The 2D scan is a flat slice through the lidar. On the ramp the robot pitches
(8.5 deg in the sim), the slice tips with it and meets the flat ground ahead:
from the pole (1.55 m up) about 10 m ahead, from the low mount about 3.7 m.
The EKF runs in two_d_mode, so TF still says the robot is level and every
consumer (costmaps, SLAM, the lane detector's /scan_fused) draws that arc of
ground as a wall across the course.

This node reads roll/pitch from the IMU, works out each return's height above
the ground in a gravity-level frame, and replaces returns below ground_margin
with inf -- but only while the robot is actually tilted (min_tilt_deg), so on
flat ground low obstacles are untouched. Everything else passes through.

    /scan_raw (lidar) --> scan_tilt_filter --> /scan (everything else)

The IMU sits on camera_link, which can itself be tilted (pole_top mount is
15 deg down), so the mounting rotation base_link -> IMU frame is taken from
TF and removed: only the robot's own tilt counts.

    ros2 run esda_simulation_2025 scan_tilt_filter.py
    python3 scan_tilt_filter.py --selftest        # no ROS needed
"""

import argparse
import math
import sys

import numpy as np


# ==========================================================================
# Pure geometry (no ROS)
# ==========================================================================

def quat_to_matrix(x, y, z, w):
    """3x3 rotation matrix from a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def level_rotation(r_world_base):
    """The robot's attitude with heading removed: rotates base-frame vectors
    into a frame whose z is straight up and whose x is the robot's heading."""
    yaw = math.atan2(r_world_base[1, 0], r_world_base[0, 0])
    c, s = math.cos(-yaw), math.sin(-yaw)
    r_unyaw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return r_unyaw @ r_world_base


def tilt_deg(r_level_base):
    """Angle between the robot's up axis and true up, degrees."""
    return math.degrees(math.acos(max(-1.0, min(1.0, r_level_base[2, 2]))))


def ground_hits(ranges, angle_min, angle_increment, r_level_laser, t_level_laser,
                base_height, ground_margin):
    """Boolean mask of returns whose point lies within ground_margin of the
    ground. r/t_level_laser place the lidar in the level frame (origin on the
    robot's base_link, which rides base_height above the ground)."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_increment * np.arange(r.size)
    finite = np.isfinite(r) & (r > 0.0)
    pts = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)])
    z = (r_level_laser @ np.nan_to_num(pts))[2] + t_level_laser[2] + base_height
    return finite & (z < ground_margin)


def selftest():
    fails = []

    def ck(name, cond):
        print("  %-62s %s" % (name, "ok" if cond else "FAIL"))
        if not cond:
            fails.append(name)

    def rot_y(p):
        c, s = math.cos(p), math.sin(p)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    def rot_z(y):
        c, s = math.cos(y), math.sin(y)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    base_h = 0.1625
    t_lidar = np.array([-0.405, 0.0, 1.385])      # pole-top laser in base_link
    n = 898
    amin, inc = -math.pi, 0.007
    ahead = int(round((0.0 - amin) / inc))
    behind = 0

    # Level robot: nothing near the ground can be seen by a flat slice.
    r_lvl = level_rotation(np.eye(3))
    ranges = [10.0] * n
    hits = ground_hits(ranges, amin, inc, r_lvl, r_lvl @ t_lidar, base_h, 0.3)
    ck("level robot: nothing dropped", not hits.any())

    # Nose down 8.5 deg (descending the ramp), heading irrelevant.
    pitch = math.radians(8.5)
    r_world_base = rot_z(1.2) @ rot_y(pitch)
    r_lvl = level_rotation(r_world_base)
    ck("tilt is read as 8.5 deg whatever the heading",
       abs(tilt_deg(r_lvl) - 8.5) < 1e-6)
    t = r_lvl @ t_lidar
    strike = (t[2] + base_h) / math.tan(pitch)    # where the slice meets the ground
    ranges = [float('inf')] * n
    ranges[ahead] = strike
    hits = ground_hits(ranges, amin, inc, r_lvl, t, base_h, 0.3)
    ck(f"ground strike straight ahead ({strike:.1f} m) is dropped", hits[ahead])
    ranges[ahead] = 5.0                            # a barrel 5 m ahead
    hits = ground_hits(ranges, amin, inc, r_lvl, t, base_h, 0.3)
    ck("a barrel 5 m ahead is kept", not hits[ahead])
    ranges[behind] = 10.0                          # behind: the slice points up
    hits = ground_hits(ranges, amin, inc, r_lvl, t, base_h, 0.3)
    ck("returns behind a nose-down robot are kept", not hits[behind])
    ranges = [float('inf')] * n
    ck("no returns, nothing dropped",
       not ground_hits(ranges, amin, inc, r_lvl, t, base_h, 0.3).any())

    print()
    print(f"{len(fails)} FAILED" if fails else "ALL CHECKS PASSED")
    return 1 if fails else 0


# ==========================================================================
# ROS node
# ==========================================================================

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu, LaserScan
    from tf2_ros import Buffer, TransformListener, TransformException

    class ScanTiltFilter(Node):
        def __init__(self):
            super().__init__('scan_tilt_filter')
            p = self.declare_parameter
            p('scan_in', '/scan_raw')
            p('scan_out', '/scan')
            p('imu_topic', '/imu/data')
            p('base_frame', 'base_link')
            p('base_height', 0.1625)      # base_link above the ground (wheel radius)
            p('ground_margin', 0.30)      # m; returns lower than this are ground
            p('min_tilt_deg', 2.0)        # only filter while tilted at least this
            p('imu_timeout', 0.5)
            g = lambda n: self.get_parameter(n).value
            self.base_frame = g('base_frame')
            self.base_height = float(g('base_height'))
            self.ground_margin = float(g('ground_margin'))
            self.min_tilt = float(g('min_tilt_deg'))
            self.imu_timeout = float(g('imu_timeout'))

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.static = {}              # frame -> (R, t) relative to base_frame
            self.r_level_base = None
            self.imu_time = None
            self.dropped_total = 0

            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
            self.pub = self.create_publisher(LaserScan, g('scan_out'), qos)
            self.create_subscription(Imu, g('imu_topic'), self.imu_cb, 50)
            self.create_subscription(LaserScan, g('scan_in'), self.scan_cb, qos)
            self.get_logger().info(
                f"{g('scan_in')} -> {g('scan_out')}: dropping ground returns "
                f"below {self.ground_margin} m while tilted > {self.min_tilt} deg")

        def mount(self, frame):
            """(R, t) of `frame` in base_frame; cached, it is a static joint."""
            if frame not in self.static:
                tr = self.tf_buffer.lookup_transform(
                    self.base_frame, frame, rclpy.time.Time()).transform
                q = tr.rotation
                self.static[frame] = (
                    quat_to_matrix(q.x, q.y, q.z, q.w),
                    np.array([tr.translation.x, tr.translation.y, tr.translation.z]))
            return self.static[frame]

        def imu_cb(self, msg):
            q = msg.orientation
            if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
                return                    # no orientation in this IMU
            try:
                r_base_imu, _ = self.mount(msg.header.frame_id)
            except TransformException:
                return
            r_world_imu = quat_to_matrix(q.x, q.y, q.z, q.w)
            self.r_level_base = level_rotation(r_world_imu @ r_base_imu.T)
            self.imu_time = self.get_clock().now()

        def scan_cb(self, msg):
            fresh = (self.r_level_base is not None and self.imu_time is not None
                     and (self.get_clock().now() - self.imu_time).nanoseconds / 1e9
                     < self.imu_timeout)
            if not fresh or tilt_deg(self.r_level_base) < self.min_tilt:
                self.pub.publish(msg)
                return
            try:
                r_base_laser, t_base_laser = self.mount(msg.header.frame_id)
            except TransformException:
                self.pub.publish(msg)
                return
            hits = ground_hits(
                msg.ranges, msg.angle_min, msg.angle_increment,
                self.r_level_base @ r_base_laser,
                self.r_level_base @ t_base_laser,
                self.base_height, self.ground_margin)
            if hits.any():
                ranges = np.asarray(msg.ranges, dtype=np.float32)
                ranges[hits] = np.inf
                msg.ranges = ranges.tolist()
                self.dropped_total += int(hits.sum())
                self.get_logger().info(
                    f"tilted {tilt_deg(self.r_level_base):.1f} deg: dropped "
                    f"{int(hits.sum())} ground returns",
                    throttle_duration_sec=2.0)
            self.pub.publish(msg)

    rclpy.init()
    node = ScanTiltFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument('--selftest', action='store_true')
    known, _ = ap.parse_known_args()
    if known.selftest:
        sys.exit(selftest())
    main()


# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# from sensor_msgs.msg import Imu, LaserScan
# from tf2_ros import Buffer, TransformListener, TransformException

# class ScanTiltFilter(Node):
#     def __init__(self):
#         super().__init__('scan_tilt_filter')

#         # Defining the scan parameters
#         self.declare_parameter('scan_publisher', '/scan')         # This the robot's SLAM with the filter scans
#         self.declare_parameter('scan_subscriber', '/scan_raw')    # This gets the raw LiDAR scan 

#         # Attaches the scan callback function
#         # self.scan_callback_timer = 