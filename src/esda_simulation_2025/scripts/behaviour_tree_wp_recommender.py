#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import py_trees
import py_trees_ros
import sys
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Waypoint Class to represent a waypoint with x, y coordinates and a name
class Waypoint:
    def __init__(self, x, y, name, yaw_w: float = 1.0):
        self.x = x
        self.y = y
        self.name = name
        self.yaw_w = yaw_w  # Yaw weight for orientation preference

class AutonomousWaypointPatrol(Node):
    def __init__(self):
        super().__init__('esda_autonomous_waypoint_patrol')
        self.get_logger().info("ESDA Autonomous Waypoint Patrol Node Initialized")

        # Initialise NAV2 command API
        self.navigator = BasicNavigator()

# 3. Main ROS2 Behaviour Tree Node
class ESDABehaviourTreeNode(Node):
    pass

def main(args=None):
    pass

# Entry point of the script
if __name__ == '__main__':
    main()