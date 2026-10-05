#!/usr/bin/env python3
"""
ODrive v3.6 bridge for the real robot (USB, one board, two axes).

  /cmd_vel (Twist)  ->  differential drive kinematics  ->  axis input_vel

Each wheel is driven through a gearbox, so a wheel turn is gear_ratio motor
turns. The ODrive works in motor turns/s; everything on the ROS side is in
wheel units (m/s, rad/s, rad).

Publishes:
  /odom              (Odometry: pose integrated from encoder positions,
                      twist from the same positions differenced per cycle)
  odom -> base_link  (TF, when publish_odom_tf is true)
  /joint_states      (JointState, wheel angles so the wheels spin in RViz)
  /odrive/wheel_velocity  (JointState, measured wheel speeds in rad/s,
                           forward positive)
  /odrive/twist      (TwistStamped, measured body velocity: linear.x m/s,
                      angular.z rad/s)

Every attribute access on the ODrive is a USB round trip costing a few ms of
Python CPU, so a cycle only reads the two encoder positions. input_vel is
written only when the target changes, and the watchdog is fed every
watchdog_feed_interval rather than every cycle.

Services:
  ~/reset_odom       (std_srvs/Trigger) zero the odom pose (x, y, yaw) so
                      base_link is back on the odom origin. Restart SLAM
                      afterwards or it sees the jump as the robot moving.

The ODrive's own config (control mode, vel_limit, ramp, current limits,
calibration) is assumed to be set up and saved already. This node only
clears errors, requests closed loop control and writes input_vel. It also
feeds the axis watchdog every cycle, so a watchdog left enabled on the board
doesn't disarm the axes (error 0x800); with it disabled that's a no-op.

Safety:
  - Wheel speeds are clamped to max_motor_turns_per_s. When one wheel
    saturates, both are scaled together so the turn radius is kept.
  - Zero velocity is commanded when /cmd_vel stops arriving for longer than
    cmd_vel_timeout.
  - An axis disarmed by an error (e.g. current limit violation) is zeroed,
    its errors cleared and put back into closed loop.
  - Both axes are put back to IDLE on shutdown.
"""

import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

# Optional: without the library the node still publishes
# odom -> base_link so the robot and LiDAR show up in RViz.
try:
    import odrive
except ImportError:
    odrive = None

# odrive.enums values for firmware 0.5.x, hardcoded so the node doesn't
# depend on which odrive Python package version exposes which names.
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8


class OdriveBridge(Node):

    def __init__(self):
        super().__init__('odrive_bridge')

        self.declare_parameter('serial_number', '')  # '' = first ODrive found
        self.declare_parameter('left_axis', 1)
        self.declare_parameter('right_axis', 0)
        # The motors are mirrored on the chassis, so one of them has to turn
        # backwards to drive forwards. Flip these if a wheel spins the wrong way.
        self.declare_parameter('left_direction', -1.0)
        self.declare_parameter('right_direction', 1.0)
        self.declare_parameter('gear_ratio', 64.0)  # motor turns per wheel turn
        self.declare_parameter('wheel_radius', 0.1552)
        self.declare_parameter('wheel_separation', 0.66)
        self.declare_parameter('max_motor_turns_per_s', 30.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('error_check_interval', 0.1)  # s between axis state checks
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('watchdog_feed_interval', 0.1)  # s, board timeout is 0.5 s
        # Low-pass on the position-differenced wheel speeds (1 = no filtering).
        self.declare_parameter('velocity_filter_alpha', 0.5)
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('left_wheel_joint', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'right_wheel_joint')

        p = lambda name: self.get_parameter(name).value
        self.serial_number = p('serial_number')
        self.axis_ids = (int(p('left_axis')), int(p('right_axis')))
        self.directions = (float(p('left_direction')), float(p('right_direction')))
        self.gear_ratio = float(p('gear_ratio'))
        self.wheel_radius = float(p('wheel_radius'))
        self.wheel_separation = float(p('wheel_separation'))
        self.max_motor_vel = float(p('max_motor_turns_per_s'))
        self.cmd_vel_timeout = float(p('cmd_vel_timeout'))
        self.error_check_interval = float(p('error_check_interval'))
        self.watchdog_feed_interval = float(p('watchdog_feed_interval'))
        self.velocity_alpha = float(p('velocity_filter_alpha'))
        self.publish_odom_tf = bool(p('publish_odom_tf'))
        self.odom_frame = p('odom_frame')
        self.base_frame = p('base_frame')
        self.joint_names = [p('left_wheel_joint'), p('right_wheel_joint')]

        self.odrv = None
        self.axes = None
        self.search_thread = None

        self.target_wheel = [0.0, 0.0]  # rad/s
        self.last_cmd_time = 0.0
        self.last_error_check = 0.0
        self.last_watchdog_feed = 0.0
        self.written_vel = None  # motor turns/s last sent per axis; None = resend
        self.reset_count = 0

        self.last_motor_pos = None  # motor turns, as read from the encoders
        self.last_pos_time = None
        self.wheel_angle = [0.0, 0.0]  # rad
        self.wheel_vel = [0.0, 0.0]  # rad/s
        self.x = self.y = self.yaw = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.wheel_vel_pub = self.create_publisher(JointState, '/odrive/wheel_velocity', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/odrive/twist', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_service(Trigger, '~/reset_odom', self.reset_odom_callback)

        self.period = 1.0 / float(p('update_rate'))
        self.create_timer(self.period, self.update)

        self.get_logger().info(
            f'ODrive bridge started: gear {self.gear_ratio}:1, '
            f'max {self.motor_to_linear(self.max_motor_vel):.2f} m/s at the wheel')
        self.start_search()

    # ---- unit conversion ----------------------------------------------

    def wheel_to_motor(self, wheel_rad_s):
        return wheel_rad_s / (2.0 * math.pi) * self.gear_ratio

    def motor_to_wheel(self, motor_turns):
        return motor_turns / self.gear_ratio * 2.0 * math.pi

    def motor_to_linear(self, motor_turns_s):
        return self.motor_to_wheel(motor_turns_s) * self.wheel_radius

    # ---- ODrive connection --------------------------------------------

    def start_search(self):
        if odrive is None:
            self.get_logger().error(
                'odrive Python library not installed (pip3 install odrive==0.5.4) - '
                'motors disabled')
            return
        if self.search_thread is not None and self.search_thread.is_alive():
            return
        self.search_thread = threading.Thread(target=self.search, daemon=True)
        self.search_thread.start()

    def search(self):
        """Background thread: block until an ODrive appears, then set it up.

        odrive.find_any() blocks, and its timeout argument is broken in
        odrive 0.5.4 (it never expires), so it can't run in the timer
        callback - update() keeps publishing odom while this waits.
        """
        serial_number = self.serial_number.upper() or None
        while rclpy.ok():
            self.get_logger().info(
                'Waiting for ODrive on USB (motors disabled until it is found)...')
            try:
                odrv = odrive.find_any(serial_number=serial_number)
                if odrv is None:
                    continue
                self.configure(odrv)
                return
            except Exception as error:  # fibre raises a variety of USB errors
                self.get_logger().error(f'ODrive setup failed: {error}')
                time.sleep(2.0)

    def configure(self, odrv):
        axes = [getattr(odrv, f'axis{i}') for i in self.axis_ids]
        self.get_logger().info(
            f'Connected to ODrive {odrv.serial_number:012X}, '
            f'fw {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}, '
            f'vbus {odrv.vbus_voltage:.1f} V')

        for name, axis in zip(('left', 'right'), axes):
            if axis.config.enable_watchdog:
                self.get_logger().info(
                    f'{name} axis watchdog enabled ({axis.config.watchdog_timeout:.2f} s), '
                    'feeding it every cycle')

        for axis in axes:
            axis.watchdog_feed()
        self.clear_errors(odrv, axes)
        for axis in axes:
            axis.watchdog_feed()
            axis.controller.input_vel = 0.0
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # update() isn't feeding yet (self.axes is still None), so split the
        # wait to stay inside short watchdog timeouts.
        for _ in range(5):
            time.sleep(0.1)
            for axis in axes:
                axis.watchdog_feed()
        for name, axis in zip(('left', 'right'), axes):
            if axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                self.get_logger().info(f'{name} axis: closed loop OK')
            else:
                self.get_logger().error(
                    f'{name} axis did not enter closed loop control '
                    f'(state {axis.current_state}): {self.describe_errors(axis)}. '
                    'Will keep retrying.')

        # Hand over to update() only once fully configured.
        self.last_motor_pos = None
        self.written_vel = None
        self.odrv = odrv
        self.axes = axes

    def disconnect(self, reason):
        self.get_logger().error(f'Lost ODrive: {reason}')
        self.odrv = None
        self.axes = None
        self.last_motor_pos = None
        self.wheel_vel = [0.0, 0.0]
        self.start_search()

    @staticmethod
    def clear_errors(odrv, axes):
        if hasattr(odrv, 'clear_errors'):  # fw >= 0.5.2
            odrv.clear_errors()
            return
        for axis in axes:
            axis.error = 0
            axis.motor.error = 0
            axis.encoder.error = 0
            axis.controller.error = 0

    @staticmethod
    def has_error(axis):
        return bool(axis.error or axis.motor.error or axis.encoder.error
                    or axis.controller.error)

    def reset_on_error(self):
        """Re-arm any axis that was disarmed by an error (current limit
        violation etc.). Its input_vel is zeroed first so it doesn't lurch."""
        failed = [(name, axis) for name, axis in zip(('left', 'right'), self.axes)
                  if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL
                  and self.has_error(axis)]
        if not failed:
            return

        self.reset_count += 1
        for name, axis in failed:
            axis.controller.input_vel = 0.0
            self.get_logger().warn(
                f'ODrive reset #{self.reset_count}: {name} axis disarmed: '
                f'{self.describe_errors(axis)}')

        self.clear_errors(self.odrv, self.axes)
        for axis in self.axes:
            axis.watchdog_feed()
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.written_vel = None  # failed axes were zeroed; resend targets

    @staticmethod
    def describe_errors(axis):
        return (f'axis=0x{axis.error:X} motor=0x{axis.motor.error:X} '
                f'encoder=0x{axis.encoder.error:X} controller=0x{axis.controller.error:X}')

    def stop_axes(self):
        if self.axes is None:
            return
        try:
            for axis in self.axes:
                axis.controller.input_vel = 0.0
                axis.requested_state = AXIS_STATE_IDLE
        except Exception as error:
            self.get_logger().warn(f'Could not idle ODrive axes: {error}')

    # ---- ROS callbacks ------------------------------------------------

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        half = self.wheel_separation / 2.0
        left = (v - w * half) / self.wheel_radius
        right = (v + w * half) / self.wheel_radius

        # Scale both wheels together if either exceeds the motor limit, so
        # the commanded turn radius is preserved.
        largest = max(abs(self.wheel_to_motor(left)), abs(self.wheel_to_motor(right)))
        if largest > self.max_motor_vel:
            scale = self.max_motor_vel / largest
            left *= scale
            right *= scale

        self.target_wheel = [left, right]
        self.last_cmd_time = time.monotonic()

    def reset_odom_callback(self, request, response):
        # Same executor as update(), so this can't interleave with an
        # odometry step. last_motor_pos is kept so the next step only adds
        # motion from here on.
        self.x = self.y = self.yaw = 0.0
        self.wheel_angle = [0.0, 0.0]
        self.get_logger().info('Odometry reset to the odom origin')
        response.success = True
        response.message = 'odom reset to x=0 y=0 yaw=0'
        return response

    def update(self):
        if self.axes is None:
            # Keep odom -> base_link and the wheel TFs alive so the robot and
            # LiDAR still show up in RViz while the ODrive is unplugged.
            self.publish_state(0.0, 0.0)
            return

        if time.monotonic() - self.last_cmd_time > self.cmd_vel_timeout:
            self.target_wheel = [0.0, 0.0]

        try:
            now = time.monotonic()
            if now - self.last_error_check > self.error_check_interval:
                self.last_error_check = now
                self.reset_on_error()

            command = [direction * self.wheel_to_motor(target)
                       for direction, target in zip(self.directions, self.target_wheel)]
            if command != self.written_vel:
                for axis, vel in zip(self.axes, command):
                    axis.controller.input_vel = vel
                self.written_vel = command

            if now - self.last_watchdog_feed > self.watchdog_feed_interval:
                self.last_watchdog_feed = now
                for axis in self.axes:
                    axis.watchdog_feed()

            motor_pos = [direction * axis.encoder.pos_estimate
                         for axis, direction in zip(self.axes, self.directions)]
            pos_time = time.monotonic()
        except Exception as error:  # ObjectLostError etc. on USB disconnect
            self.disconnect(error)
            return

        linear, angular = self.update_odometry(motor_pos, pos_time)
        self.publish_measured_velocity(linear, angular)

    def publish_measured_velocity(self, linear, angular):
        stamp = self.get_clock().now().to_msg()

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.joint_names
        msg.velocity = list(self.wheel_vel)
        self.wheel_vel_pub.publish(msg)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = self.base_frame
        twist.twist.linear.x = linear
        twist.twist.angular.z = angular
        self.twist_pub.publish(twist)

    # ---- odometry -----------------------------------------------------

    def update_odometry(self, motor_pos, pos_time):
        """Integrate the pose from encoder positions, and get wheel speeds
        by differencing them over the real time between reads (lightly
        low-pass filtered). Returns (linear, angular).
        """
        if self.last_motor_pos is None:
            self.last_motor_pos = motor_pos
            self.last_pos_time = pos_time
            self.wheel_vel = [0.0, 0.0]
            self.publish_state(0.0, 0.0)
            return 0.0, 0.0

        d_wheel = [self.motor_to_wheel(now - before)
                   for now, before in zip(motor_pos, self.last_motor_pos)]
        dt = pos_time - self.last_pos_time
        self.last_motor_pos = motor_pos
        self.last_pos_time = pos_time

        if dt > 0.0:
            a = self.velocity_alpha
            self.wheel_vel = [a * (d / dt) + (1.0 - a) * v
                              for d, v in zip(d_wheel, self.wheel_vel)]  # rad/s
        linear = (self.wheel_vel[0] + self.wheel_vel[1]) / 2.0 * self.wheel_radius
        angular = ((self.wheel_vel[1] - self.wheel_vel[0]) * self.wheel_radius
                   / self.wheel_separation)

        for i in range(2):
            self.wheel_angle[i] += d_wheel[i]

        d_left = d_wheel[0] * self.wheel_radius
        d_right = d_wheel[1] * self.wheel_radius
        d_dist = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / self.wheel_separation

        # Midpoint integration
        mid_yaw = self.yaw + d_yaw / 2.0
        self.x += d_dist * math.cos(mid_yaw)
        self.y += d_dist * math.sin(mid_yaw)
        self.yaw = math.atan2(math.sin(self.yaw + d_yaw), math.cos(self.yaw + d_yaw))

        self.publish_state(linear, angular)
        return linear, angular

    def publish_state(self, linear, angular):
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        for i, value in zip((0, 7, 14, 21, 28, 35), (0.001, 0.001, 1e6, 1e6, 1e6, 0.01)):
            odom.pose.covariance[i] = value
        for i, value in zip((0, 7, 14, 21, 28, 35), (0.001, 1e6, 1e6, 1e6, 1e6, 0.01)):
            odom.twist.covariance[i] = value
        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.joint_names
        msg.position = list(self.wheel_angle)
        msg.velocity = list(self.wheel_vel)
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdriveBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_axes()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
