#!/usr/bin/env python3
"""What do the sensors do when the robot is on the ramp?

    python3 ramp_probe.py --out /tmp/ramp.csv --seconds 600

Every run on igvc.sdf dies between y = 28.3 and 30.3 -- the ramp, two 4x2 m
plates pitched ~8.5 deg, peaking about 0.30 m at y = 29.3. Lane detection
is fine up there (both boundaries 43-56%, same as everywhere) and the
navigator reports stuck, backs out and retries. So the question is what
the sensors say, not what the detector sees.

The specific thing to catch: the LiDAR is a horizontal 2D scan on a rigid
mount. On the ramp the robot pitches, and a 2D scanner that pitches DOWN
paints the ground ahead as an obstacle. At 0.385 m and 8.5 deg down the
ground is 2.6 m away, which is well inside obstacle_max_range and lands in
the costmap as a wall across the course.

PITCH: USE THE EKF, NOT THE RAW IMU. The first version of this read
/imu/data directly and reported +19 deg on flat ground, pinned at +30.00
everywhere. That IMU is the ZED's, on the pole mount behind a 15 deg tilt
and the optical-frame rotations, so its quaternion is the sensor's
attitude, not the robot's. /odometry/filtered comes out of robot_localization
already in base_link, which is the frame the question is about. Both are
logged so the difference stays visible rather than being quietly dropped.

COMMANDED VERSUS ACTUAL. The thing that separates "cannot climb" from
"was never told to climb": if cmd_vel asks for forward motion and the
wheels report none, it is traction; if cmd_vel is near zero, the
navigator stopped asking and the slope is not the problem.

Logs at 5 Hz: pose, both pitches, commanded and actual speed, and the
nearest return ahead on /scan and /scan_fused, which is what Nav2
consumes. Then the rows are grouped by
where the robot was, so "on the ramp" can be compared with "before it"
without any guessing about timing.
"""
import argparse
import math
import os
import sys
import time


def quat_pitch(x, y, z, w):
    """Pitch in degrees, +ve nose up."""
    s = 2.0 * (w * y - z * x)
    s = max(-1.0, min(1.0, s))
    return math.degrees(math.asin(s))


def min_ahead(ranges, angle_min, angle_inc, half_deg, rmin=0.05, rmax=40.0):
    """Nearest finite return within +/-half_deg of straight ahead."""
    best = float("inf")
    half = math.radians(half_deg)
    for i, r in enumerate(ranges):
        if r is None or not (rmin < r < rmax):
            continue
        a = angle_min + i * angle_inc
        if -half <= a <= half and r < best:
            best = r
    return best


def selftest():
    fails = []

    def ck(n, c):
        print("  %-52s %s" % (n, "ok" if c else "FAIL"))
        if not c:
            fails.append(n)

    ck("level quaternion is 0 pitch", abs(quat_pitch(0, 0, 0, 1)) < 1e-9)
    p = math.radians(10.0)
    ck("10 deg nose-up reads +10",
       abs(quat_pitch(0, math.sin(p / 2), 0, math.cos(p / 2)) - 10.0) < 1e-6)
    ck("10 deg nose-down reads -10",
       abs(quat_pitch(0, math.sin(-p / 2), 0, math.cos(-p / 2)) + 10.0) < 1e-6)

    rr = [float("inf")] * 10 + [2.0] + [float("inf")] * 10
    ck("finds the one real return straight ahead",
       abs(min_ahead(rr, -math.radians(10) * 1.0, math.radians(1.0), 15) - 2.0) < 1e-9)
    ck("ignores a return outside the cone",
       min_ahead([1.0] + [float("inf")] * 20, math.radians(-60),
                 math.radians(1.0), 15) == float("inf"))
    ck("ignores non-finite and out-of-range",
       min_ahead([float("nan"), 0.0, 100.0], math.radians(-1),
                 math.radians(1.0), 15) == float("inf"))
    # commanded-versus-actual, the distinction the whole run turns on
    def verdict(cmd, act, eps_cmd=0.05, eps_act=0.03):
        if abs(cmd) < eps_cmd:
            return "not commanded"
        return "traction" if abs(act) < eps_act else "moving"
    ck("asked to move and not moving -> traction", verdict(0.4, 0.00) == "traction")
    ck("asked to move and moving -> moving", verdict(0.4, 0.35) == "moving")
    ck("not asked -> not commanded", verdict(0.0, 0.00) == "not commanded")
    ck("a crawl still counts as moving", verdict(0.4, 0.10) == "moving")

    print()
    if fails:
        print("%d FAILED" % len(fails)); return 1
    print("ALL CHECKS PASSED"); return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--out", default="/tmp/ramp_probe.csv")
    ap.add_argument("--seconds", type=float, default=600.0)
    ap.add_argument("--odom", default="/diff_drive_base_controller/odom")
    ap.add_argument("--hz", type=float, default=5.0)
    a = ap.parse_args()
    if a.selftest:
        return selftest()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from geometry_msgs.msg import Twist

    class Probe(Node):
        def __init__(self):
            super().__init__("ramp_probe")
            self.x = self.y = 0.0
            self.pitch_ekf = float("nan")
            self.pitch_imu = float("nan")
            self.cmd_vx = 0.0
            self.act_vx = 0.0
            self.scan = {}
            self.n = 0
            self.f = open(a.out, "w")
            self.f.write("t,x,y,pitch_ekf,pitch_imu,cmd_vx,act_vx,min_scan,min_fused\n")
            self.create_subscription(Odometry, a.odom, self._odom, 10)
            self.create_subscription(Odometry, "/odometry/filtered",
                                     self._ekf, 10)
            self.create_subscription(Imu, "/imu/data", self._imu,
                                     qos_profile_sensor_data)
            for t in ("/cmd_vel", "/cmd_vel_nav"):
                self.create_subscription(
                    Twist, t, (lambda tt: lambda m: self._cmd(tt, m))(t), 10)
            for t in ("/scan", "/scan_fused"):
                self.create_subscription(
                    LaserScan, t,
                    (lambda tt: lambda m: self._scan(tt, m))(t),
                    qos_profile_sensor_data)
            self.create_timer(1.0 / a.hz, self._tick)
            self.t0 = time.time()
            self.get_logger().info("logging -> %s" % a.out)

        def _odom(self, m):
            self.x = m.pose.pose.position.x
            self.y = m.pose.pose.position.y
            self.act_vx = m.twist.twist.linear.x

        def _ekf(self, m):
            q = m.pose.pose.orientation
            self.pitch_ekf = quat_pitch(q.x, q.y, q.z, q.w)

        def _cmd(self, t, m):
            # whichever is moving; twist_mux means only one is live
            if abs(m.linear.x) > abs(self.cmd_vx) or t == "/cmd_vel":
                self.cmd_vx = m.linear.x

        def _imu(self, m):
            q = m.orientation
            self.pitch_imu = quat_pitch(q.x, q.y, q.z, q.w)

        def _scan(self, t, m):
            self.scan[t] = min_ahead(list(m.ranges), m.angle_min,
                                     m.angle_increment, 15.0)

        def _tick(self):
            self.f.write("%.2f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\n" % (
                time.time() - self.t0, self.x, self.y,
                self.pitch_ekf, self.pitch_imu, self.cmd_vx, self.act_vx,
                self.scan.get("/scan", float("inf")),
                self.scan.get("/scan_fused", float("inf"))))
            self.cmd_vx = 0.0          # decay, so a stale command cannot
                                       # look like a live one
            self.f.flush()
            self.n += 1

    rclpy.init()
    n = Probe()
    end = time.time() + a.seconds
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        n.f.close()
        print("[probe] %d rows -> %s" % (n.n, a.out))
        n.destroy_node()
        rclpy.shutdown()
    return 0 if n.n else 1


if __name__ == "__main__":
    raise SystemExit(main())
