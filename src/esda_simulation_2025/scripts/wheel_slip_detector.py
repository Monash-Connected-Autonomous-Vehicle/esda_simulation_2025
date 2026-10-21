"""
Wheel slip detection node for the real robot. 
This node subscribes to the odometry topic and the path topic, 
and detects wheel slip based on the difference between the expected path and the actual odometry data. 
If wheel slip is detected, it will publish a warning message.

This node compares encoder data against gyro data to detect wheel slip.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import customtkinter as ctk
import threading
import time
import json
import os
from datetime import datetime


class WheelSlipDetector(Node):
    def __init__(self):
        super().__init__('wheel_slip_detector')
        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('gyro_topic', '/gyro')
        