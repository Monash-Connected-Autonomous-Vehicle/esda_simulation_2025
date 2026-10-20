#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Replace with your required message type
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class ThrottlePublisher(Node):
    def __init__(self):
        super().__init__('throttle_publisher')
        self.timer_period = 1.0  # seconds
        self.last_published_time = self.get_clock().now()
        self.max_throttle_speed = 2.2  # Replace with your actual max throttle speed
        self.max_throttle = 1.0

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,  # Replace with your required message type
            '/cmd_vel',  # Replace with your topic name
            self.cmd_vel_callback,
            10
        )

        # Publish calculated throttle command (right)
        self.right_throttle_publisher = self.create_publisher(
            Float32,
            '/right_throttle',
            10
        )
        
        # Publish calculated throttle command (left)
        self.left_throttle_publisher = self.create_subscription(
            Float32,
            '/left_throttle',
            10
        )

        self.get_logger().info('Throttle Publisher Node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = 'Throttle message'  # Replace with your actual message content
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def cmd_vel_callback(self, msg):
        # Requested forward velocity from /cmd_vel
        linear_x = msg.linear.x

        # Convert velocity into normalized throttle
        throttle = (
            linear_x / self.max_throttle_speed
        ) * self.max_throttle

        # Clamp throttle between -1.0 and 1.0
        throttle = max(
            -self.max_throttle,
            min(self.max_throttle, throttle)
        )

        # Create throttle message
        throttle_msg = Float32()
        throttle_msg.data = float(throttle)

        # Publish throttle
        self.throttle_publisher.publish(throttle_msg)

        self.get_logger().info(
            f'cmd_vel linear.x: {linear_x:.3f} m/s -> '
            f'throttle: {throttle:.3f}'
        )

if __name__ == '__main__':
    rclpy.init()
    node = ThrottlePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
