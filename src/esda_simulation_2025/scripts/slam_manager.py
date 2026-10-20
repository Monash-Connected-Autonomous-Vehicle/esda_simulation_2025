#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

import math
import time

class SlamManager(Node):
    def __init__(self):
        super().__init__('slam_manager')

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular

        speed = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
        
