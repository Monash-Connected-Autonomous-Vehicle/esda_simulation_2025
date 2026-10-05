#!/usr/bin/env python3
"""
Live terminal view of what the ODrive robot is told vs what it does.

  /cmd_vel                 commanded linear/angular, plus the per-wheel
                           targets odrive_bridge.py derives from it
  /odrive/twist            measured body velocity (encoder vel_estimate)
  /odrive/wheel_velocity   measured wheel speeds
  /odom                    integrated pose

Read-only: it never publishes, so it's safe to run next to teleop or Nav2.
The kinematics parameters must match odrive_bridge.py for the "target"
columns to line up with the measured ones.

    ros2 run esda_simulation_2025 odrive_monitor.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

STALE_AFTER = 0.5  # s, same as the bridge's cmd_vel_timeout


class OdriveMonitor(Node):

    def __init__(self):
        super().__init__('odrive_monitor')

        self.declare_parameter('gear_ratio', 64.0)
        self.declare_parameter('wheel_radius', 0.1552)
        self.declare_parameter('wheel_separation', 0.66)
        self.declare_parameter('max_motor_turns_per_s', 30.0)
        self.declare_parameter('refresh_rate', 5.0)

        p = lambda name: float(self.get_parameter(name).value)
        self.gear_ratio = p('gear_ratio')
        self.wheel_radius = p('wheel_radius')
        self.wheel_separation = p('wheel_separation')
        self.max_motor_vel = p('max_motor_turns_per_s')

        self.cmd = None
        self.cmd_time = 0.0
        self.cmd_count = 0
        self.cmd_rate = 0.0
        self.rate_window_start = time.monotonic()

        self.twist = None
        self.twist_time = 0.0
        self.wheel_vel = None
        self.odom = None

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(TwistStamped, '/odrive/twist', self.twist_callback, 10)
        self.create_subscription(JointState, '/odrive/wheel_velocity', self.wheel_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_timer(1.0 / p('refresh_rate'), self.draw)

    # ---- callbacks ----------------------------------------------------

    def cmd_callback(self, msg):
        self.cmd = msg
        self.cmd_time = time.monotonic()
        self.cmd_count += 1

    def twist_callback(self, msg):
        self.twist = msg.twist
        self.twist_time = time.monotonic()

    def wheel_callback(self, msg):
        if len(msg.velocity) >= 2:
            self.wheel_vel = list(msg.velocity[:2])

    def odom_callback(self, msg):
        self.odom = msg

    # ---- kinematics (mirrors odrive_bridge.cmd_vel_callback) ----------

    def wheel_targets(self, v, w):
        """Return (left m/s, right m/s, left motor t/s, right motor t/s, clamped)."""
        half = self.wheel_separation / 2.0
        left = v - w * half
        right = v + w * half
        to_motor = lambda m_s: m_s / self.wheel_radius / (2.0 * math.pi) * self.gear_ratio
        largest = max(abs(to_motor(left)), abs(to_motor(right)))
        clamped = largest > self.max_motor_vel
        if clamped:
            scale = self.max_motor_vel / largest
            left *= scale
            right *= scale
        return left, right, to_motor(left), to_motor(right), clamped

    # ---- display ------------------------------------------------------

    def draw(self):
        now = time.monotonic()
        if now - self.rate_window_start >= 1.0:
            self.cmd_rate = self.cmd_count / (now - self.rate_window_start)
            self.cmd_count = 0
            self.rate_window_start = now

        cmd_fresh = self.cmd is not None and now - self.cmd_time < STALE_AFTER
        v = self.cmd.linear.x if cmd_fresh else 0.0
        w = self.cmd.angular.z if cmd_fresh else 0.0
        tl, tr, tl_motor, tr_motor, clamped = self.wheel_targets(v, w)

        max_linear = (self.max_motor_vel / self.gear_ratio * 2.0 * math.pi
                      * self.wheel_radius)

        lines = ['ODrive monitor  (Ctrl+C to quit)', '=' * 62]

        # Command
        if self.cmd is None:
            cmd_state = 'never received'
        elif cmd_fresh:
            cmd_state = f'live, {self.cmd_rate:4.1f} Hz'
        else:
            cmd_state = f'STALE {now - self.cmd_time:5.1f} s -> bridge commands 0'
        lines.append(f'/cmd_vel        [{cmd_state}]')
        lines.append(f'  commanded     linear {v:+6.3f} m/s     angular {w:+6.3f} rad/s')
        lines.append(f'  wheel target  left   {tl:+6.3f} m/s     right   {tr:+6.3f} m/s')
        lines.append(f'  motor target  left   {tl_motor:+6.1f} t/s     right   {tr_motor:+6.1f} t/s'
                     + ('   CLAMPED' if clamped else ''))
        lines.append(f'  limits        {max_linear:.3f} m/s straight, '
                     f'{max_linear / (self.wheel_separation / 2.0):.3f} rad/s spinning')
        lines.append('')

        # Measured
        if self.twist is None:
            lines.append('/odrive/twist   [no data - is odrive_bridge connected?]')
        else:
            age = now - self.twist_time
            state = 'live' if age < STALE_AFTER else f'STALE {age:5.1f} s'
            ml = self.twist.linear.x
            ma = self.twist.angular.z
            lines.append(f'/odrive/twist   [{state}]')
            lines.append(f'  measured      linear {ml:+6.3f} m/s     angular {ma:+6.3f} rad/s')
            lines.append(f'  error         linear {ml - v:+6.3f} m/s     angular {ma - w:+6.3f} rad/s')
        if self.wheel_vel is not None:
            wl, wr = (rad_s * self.wheel_radius for rad_s in self.wheel_vel)
            ml_motor, mr_motor = (rad_s / (2.0 * math.pi) * self.gear_ratio
                                  for rad_s in self.wheel_vel)
            lines.append(f'  wheel         left   {wl:+6.3f} m/s     right   {wr:+6.3f} m/s')
            lines.append(f'  motor         left   {ml_motor:+6.1f} t/s     right   {mr_motor:+6.1f} t/s')
        lines.append('')

        # Pose
        if self.odom is not None:
            pos = self.odom.pose.pose.position
            q = self.odom.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            lines.append(f'/odom           x {pos.x:+7.3f} m   y {pos.y:+7.3f} m   '
                         f'yaw {math.degrees(yaw):+7.1f} deg')
        else:
            lines.append('/odom           [no data]')

        # Clear screen, home cursor, redraw.
        print('\033[2J\033[H' + '\n'.join(lines), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = OdriveMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
