#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import py_trees
import py_trees_ros
import sys
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan

def create_tree(navigator):
    # Create the root of the behaviour tree
    root = py_trees.composites.Sequence(name="ESDA Behaviour Tree", memory=True)

    # Create the ObstacleDetected behaviour
    obstacle_sequence = py_trees.composites.Sequence(name="Obstacle Sequence")
    obstacle_sequence.add_children([
        ObstacleDetected(name="Obstacle Detected"),
        GenerateObstacleGoal(name="Generate Obstacle Goal")]
        
    )

    # Create the LanesDetected behaviour
    lane_sequence = py_trees.composites.Sequence(name="Lane Sequence")
    lane_sequence.add_children([
        LanesDetected(name="Lanes Detected"),
        GenerateLaneGoal(name="Generate Lane Goal")]
    )


    return root

# Waypoint Class to represent a waypoint with x, y coordinates and a name
class Waypoint:
    def __init__(self, x, y, name, yaw_w: float = 1.0):
        self.x = x
        self.y = y
        self.name = name
        self.yaw_w = yaw_w  # Yaw weight for orientation preference

class ObstacleDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name = "ObstacleDetected"):
        super().__init__(name)
        self.obstacle_detected = False  # Placeholder for actual obstacle detection logic
        self.subscription = None
        self.publisher = None
        self.node = None



    def setup(self, **kwargs):
        self.node = kwargs["node"]

        self.laser_scan_subscriber = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.local_costmap_subscriber = self.node.create_subscription(
            Float32,
            '/local_costmap',
            self.costmap_callback,
            10
        )
    
    def scan_callback(self, msg):
        if len(msg.ranges) > 0:
            # Check if any range is below a certain threshold (e.g., 1.0 meter)
            self.obstacle_detected = any(range < 1.0 for range in msg.ranges)
            pass
    
    def costmap_callback(self, msg):
        # Placeholder for processing local costmap data
        # You can implement logic to determine if an obstacle is detected based on the costmap
        pass

    def update(self):
        return super().update()
    
class GenerateObstacleGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name = "GenerateObstacleGoal"):
        super().__init__(name)
        self.goal_pose = None  # Placeholder for the generated goal pose
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]

    def update(self):
        if self.goal_pose is None:
            # Generate a new goal pose based on the detected obstacle
            self.goal_pose = PoseStamped()
            self.goal_pose.header.frame_id = "map"
            self.goal_pose.pose.position.x = 1.0  # Example x coordinate
            self.goal_pose.pose.position.y = 1.0  # Example y coordinate
            self.goal_pose.pose.orientation.w = 1.0  # No rotation
            self.node.get_logger().info("Generated new goal pose based on detected obstacle.")

        return py_trees.common.Status.SUCCESS
    pass

class LanesDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name = "LanesDetected"):
        super().__init__(name)
        self.lanes_detected = False  # Placeholder for actual lane detection logic
        self.subscription = None
        self.publisher = None
        self.node = None

    def setup(self, **kwargs):
        pass


class GenerateLaneGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name = "GenerateLaneGoal"):
        super().__init__(name)
        self.goal_pose = None  # Placeholder for the generated goal pose
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]

    def update(self):
        if self.goal_pose is None:
            # Generate a new goal pose based on the detected lanes
            self.goal_pose = PoseStamped()
            self.goal_pose.header.frame_id = "map"
            self.goal_pose.pose.position.x = 1.0  # Example x coordinate
            self.goal_pose.pose.position.y = 1.0  # Example y coordinate
            self.goal_pose.pose.orientation.w = 1.0  # No rotation
            self.node.get_logger().info("Generated new goal pose based on detected lanes.")

        return py_trees.common.Status.SUCCESS

class NavigateToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, navigator, name = "NavigateToGoal"):
        super().__init__(name)
        # Navigator instance to handle navigation tasks
        self.navigator = navigator
        self.node = None

        # Goal state variables
        self.goal_active = False
        self.goal_pose = None
        self.goal_reached = False


    def setup(self, **kwargs):
        self.navigator = kwargs["navigator"]
        self.node = kwargs["node"]
    
    def update(self):
        if self.node.current_goal is None:
            self.node.get_logger().info("No goal to navigate to.")
            return py_trees.common.Status.FAILURE
        
        if not self.goal_active:
            # Send the goal to the navigator
            self.navigator.goToPose(self.node.current_goal)
            self.goal_active = True
            self.node.get_logger().info("Navigating to goal...")
            return py_trees.common.Status.RUNNING
        
        if not self.navigator.isTaskComplete():
            # Still navigating
            return py_trees.common.Status.RUNNING
        
        result = self.navigator.getResult()
        self.goal_active = False
        
        if result == TaskResult.SUCCEEDED:
            self.node.get_logger().info("Goal reached successfully.")
            return py_trees.common.Status.SUCCESS


        self.node.get_logger().info("Failed to reach the goal.")
        return py_trees.common.Status.FAILURE


# 3. Main ROS2 Behaviour Tree Node
class ESDABehaviourTreeNode(Node):
    def __init__(self):
        super().__init__('esda_behaviour_tree_node')

        self.get_logger().info("ESDA Behaviour Tree Node Initialized")

        self.navigator = BasicNavigator()
        
        root = create_tree(self.navigator)
        
        # Shared goal state between behaviours
        self.current_goal = None

        self.tree = py_trees_ros.trees.BehaviourTree(root)
        self.tree.setup(timeout=15, node=self, navigator=self.navigator)

        self.timer = self.create_timer(0.5, self.tick_tree)

    def tick_tree(self):
        self.tree.tick()
        

def main(args=None):
    pass

# Entry point of the script
if __name__ == '__main__':
    main()