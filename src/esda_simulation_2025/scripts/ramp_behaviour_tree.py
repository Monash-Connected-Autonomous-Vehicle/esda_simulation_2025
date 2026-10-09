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

class RampBehaviourTree(Node):
    def __init__(self):
        super().__init__('ramp_behaviour_tree')
        self.navigator = BasicNavigator()
        self.tree = self.create_tree(self.navigator)
        self.setup_tree()

    def create_tree(self, navigator):
        # Create the root of the behaviour tree
        root = py_trees.composites.Sequence(name="Ramp Behaviour Tree", memory=True)

        # Create the RampDetected behaviour
        ramp_sequence = py_trees.composites.Sequence(name="Ramp Sequence")
        ramp_sequence.add_children([
            RampDetected(name="Ramp Detected"),
            GenerateRampGoal(name="Generate Ramp Goal")]
        )

        root.add_child(ramp_sequence)
        return root

    def setup_tree(self):
        # Setup the behaviour tree with ROS2 integration
        py_trees_ros.trees.BehaviourTree(self.tree).setup(timeout=15)

    def tick(self):
        # Tick the behaviour tree
        py_trees_ros.trees.BehaviourTree(self.tree).tick_tock(500)  # Tick every 500ms

class RampDetected(py_trees.behaviour.Behaviour):
    pass

class GenerateRampGoal(py_trees.behaviour.Behaviour):
    pass