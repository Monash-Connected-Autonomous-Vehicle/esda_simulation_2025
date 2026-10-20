#!/usr/bin/env python3

# This node will implement a behaviour tree to manage the different navigation strategies for the robot. It will subscribe to the necessary topics to determine the current state of the robot and the environment, and will switch between different navigation strategies (e.g. follow the gap, goal navigation) based on the current state and the distance to the goal.
# The behaviour tree will be structured as follows:
# - Cases with lanes:
#   - Centreline following: If the robot detects lane markings and is not close to the goal, it will follow the centreline of the track using a lane following algorithm.
#   - LiDAR says clear --> Robot follows centreline
# - Cases where obstacle appears in front of the robot:
#   - Follow the gap: If the robot detects an obstacle in front of it, it will switch to the follow the gap algorithm to navigate around the obstacle until it is clear again, at which point it will switch back to centreline following.
#   - LiDAR says blocked --> Robot follows the gap
# - Cases where no lane is detected --> No centreline following, robot follows the gap
#   - Fallback to FTG: If the robot does not detect any lane markings, it will default to using the follow the gap algorithm to navigate until it detects lanes again or gets close to the goal.
# - Cases where the robot is close to the goal:
#   - Goal navigation: If the robot is within a certain distance to the goal, it will switch to a goal navigation strategy (e.g. using the Nav2 stack) to navigate directly towards the goal.
#   - Distance to goal < threshold --> Robot switches to goal navigation

# Ultimately, this code basically combines the lane following and follow the gap algorithms into a single node that can switch between the two based on the current state of the robot and the environment. It also incorporates a goal navigation strategy for when the robot is close to the goal, allowing it to navigate directly towards the goal when appropriate.
# FTG for close objects
# track_follower for when lanes are detected and no close obstacles for general palnner

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
import numpy as np
import math

import rclpy
from rclpy.node import Node

from enum import Enum

import tf_transformations

# from build.esda_simulation_2025.rosidl_generator_py.esda_simulation_2025.msg._navigation_recommendation import NavigationRecommendation
from esda_simulation_2025.msg import NavigationRecommendation

# Define an enumeration for the different navigation states of the robot. This will help us manage the different navigation strategies in a clear and organized way, allowing us to easily switch between them based on the current state of the robot and the environment.
class NavigationState(Enum):
    CENTRELINE_FOLLOWING = 1
    CENTRELINE_FOLLOWING_HEADING_CORRECTION = 2 
    FOLLOW_THE_GAP = 3
    GOAL_NAVIGATION = 4
    RECOVERY = 5  # Added a recovery state for handling situations where the robot is stuck or encounters an unexpected situation


class BehaviourTree(Node):
    def __init__(self):
        super().__init__('behaviour_tree')

        # Subscribing to different topics
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('frame_id', 'map')   

        self.declare_parameter('cone_near_distance', 1.4)
        self.declare_parameter('cone_far_distance', 1.8)
        self.declare_parameter('cone_front_half_angle_deg', 35.0)
        self.declare_parameter('cone_hold_time_sec', 1.0)

        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.cone_near_distance = self.get_parameter('cone_near_distance').get_parameter_value().double_value
        self.cone_far_distance = self.get_parameter('cone_far_distance').get_parameter_value().double_value
        self.cone_front_half_angle = math.radians(
            self.get_parameter('cone_front_half_angle_deg').get_parameter_value().double_value
        )
        self.cone_hold_time_sec = self.get_parameter('cone_hold_time_sec').get_parameter_value().double_value

        # Subscribes to the track follower node
        self.track_follower_subscription = self.create_subscription(
            NavigationRecommendation, 
            '/track_follower_recommendation', 
            self.track_follower_callback, 
            10
        )

        # Subscribes to the follow the gap node
        self.follow_the_gap_subscription = self.create_subscription(
            NavigationRecommendation, 
            '/follow_the_gap_recommendation', 
            self.follow_the_gap_callback, 
            10
        )

        # Subscribes to Odometry to get the current position of the robot and calculate the distance to the goal, which will help us determine when to switch to the goal navigation strategy as we get closer to the goal.
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/diff_drive_base_controller/odom',
            self.odom_callback,
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.scan_callback,
            10
        )

        


        # Latest NavigationRecommendation messages from the track follower and follow the gap nodes, which we will use in our control loop to make decisions about which navigation strategy to use based on the current state of the robot and the environment.
        self.latest_track_recommendation_msg = None
        self.latest_follow_the_gap_recommendation_msg = None

        # Latest NavigationRecommendation timestamps to track the freshness of the data and ensure we are making decisions based on the most recent information available from the track follower and follow the gap nodes.
        self.latest_track_recommendation_timestamp = None
        self.latest_follow_the_gap_recommendation_timestamp = None

        # Start the robot in the follow the gap state by default. We will switch to other states based on the sensor data and the distance to the goal.
        self.current_state = NavigationState.FOLLOW_THE_GAP

        # Declare an angle that the recovery spin should rotate to, this will be used in the recovery strategy to help the robot get unstuck or handle unexpected situations by rotating in place to try to find a clear path forward.
        self.declare_parameter('recovery_spin_angle', math.radians(35.0))  # Rotate 35 degrees in the recovery strategy to try to get unstuck or find a clear path forward
        self.recovery_spin_angle = self.get_parameter('recovery_spin_angle').get_parameter_value().double_value

        # Defining the timer control loop period
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.max_age_time = 0.5  # seconds, the maximum age of the NavigationRecommendation messages that we will consider valid for making decisions in our control loop. If the messages are older than this threshold, we will consider them stale and may choose to switch to a different navigation strategy or enter a recovery state to handle the situation appropriately.

        self.initial_yaw = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.cone_nearby = False
        self.min_front_distance = float('inf')
        self.last_cone_detection_time = self.get_clock().now()

        self.start_time = self.get_clock().now()

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return

        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max
        ranges = np.clip(ranges, msg.range_min, msg.range_max)

        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        front_mask = np.abs(angles) <= self.cone_front_half_angle
        front_ranges = ranges[front_mask]

        if front_ranges.size == 0:
            return

        self.min_front_distance = float(np.min(front_ranges))
        now = self.get_clock().now()

        if self.min_front_distance <= self.cone_near_distance:
            self.cone_nearby = True
            self.last_cone_detection_time = now
            return

        if self.min_front_distance >= self.cone_far_distance:
            time_since_detection = (now - self.last_cone_detection_time).nanoseconds / 1e9
            if time_since_detection >= self.cone_hold_time_sec:
                self.cone_nearby = False

    def follow_the_gap_msg_is_fresh(self):
        if self.latest_follow_the_gap_recommendation_timestamp is None:
            return False

        age = (self.get_clock().now() - self.latest_follow_the_gap_recommendation_timestamp).nanoseconds / 1e9
        return age < self.max_age_time

    def odom_callback(self, msg):
        # This callback will be called whenever a new Odometry message is received. We will use this information to calculate the distance to the goal and determine when to switch to the goal navigation strategy as we get closer to the goal.
        orientation = msg.pose.pose.orientation

        _, _, self.current_yaw = tf_transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        self.get_logger().info(
            f'Received Odometry: current_yaw={math.degrees(self.current_yaw):.2f} degrees'
        )

        self.odom_received = True

    def publish_cmd_vel(self, linear_x, angular_z):
        # This function will be used to publish cmd_vel messages to control the robot's movement. We will call this function from our control loop based on the current navigation strategy and the recommendations received from the track follower and follow the gap nodes.
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
        cmd_vel_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info(
            f'Published cmd_vel: linear_x={linear_x:.2f} m/s, angular_z={angular_z:.2f} rad/s'
        )

    def should_enter_recovery_mode(self):
        # This function will determine whether the robot should enter the recovery mode based on the current state and the latest sensor data. We will implement logic here to check if the robot is stuck or encounters an unexpected situation, and if so, we will return True to indicate that we should enter the recovery mode.

        # Check if there is an obstacle detected directly in front such that the robot can't see a clear path forward, and if so, we will return True to indicate that we should enter the recovery mode. This will help us handle situations where the robot is stuck or encounters an unexpected situation by switching to a recovery strategy to try to get unstuck or find a clear path forward.
        if self.latest_follow_the_gap_recommendation_msg is not None and self.latest_follow_the_gap_recommendation_msg.reason == NavigationRecommendation.RECOVERY_REQUIRED:
            self.publish_cmd_vel(0.0, 0.0)  # Stop the robot before entering recovery mode



            return True
        
        return False

    
    def control_loop(self):
        # This is the main control loop that will be called periodically by the timer. In this loop, we will check the current state of the robot and the environment based on the latest sensor data, and we will switch between different navigation strategies accordingly. We will also implement a recovery strategy for handling situations where the robot is stuck or encounters an unexpected situation.
        
        # TODO: Implement logic to check the distance to the goal and switch to goal navigation strategy when close to the goal. This will allow the robot to navigate directly towards the goal when appropriate, based on the current state and the distance to the goal.
        # Check if we need to enter the recovery mode
        if self.should_enter_recovery_mode() and self.get_clock().now() - self.start_time > rclpy.duration.Duration(seconds=15.0):
            self.get_logger().info('Entering recovery mode')
            self.current_state = NavigationState.RECOVERY    
            return

        if self.latest_follow_the_gap_recommendation_msg is not None and self.latest_follow_the_gap_recommendation_msg.confidence > 0.4:
            self.cone_nearby = True

        # Global near-obstacle override: always prioritize Follow The Gap when obstacle is near.
        if self.cone_nearby and self.current_state != NavigationState.FOLLOW_THE_GAP:
            self.get_logger().info(
                f'Near obstacle detected (min_front={self.min_front_distance:.2f} m). Forcing FOLLOW_THE_GAP.'
            )
            self.current_state = NavigationState.FOLLOW_THE_GAP
            return

        # Global safety: if both recommendation sources are too old/missing, emit safe forward motion
        track_is_fresh = (
            self.latest_track_recommendation_msg is not None 
            and self.latest_track_recommendation_timestamp is not None
            and (self.get_clock().now() - self.latest_track_recommendation_timestamp).nanoseconds / 1e9 < self.max_age_time
        )
        ftg_is_fresh = self.follow_the_gap_msg_is_fresh()


        if self.current_state == NavigationState.CENTRELINE_FOLLOWING:
            # Implement logic for centreline following navigation strategy
            # self.latest_track_recommendation_msg
            self.get_logger().info('Current state: CENTRELINE_FOLLOWING')
            
            # Gets the track_follower data:
            if self.latest_track_recommendation_msg is not None:
                self.get_logger().info(f'<DEBUG LINE 141> Latest Track Follower recommendation: valid={self.latest_track_recommendation_msg.valid}, reason={self.latest_track_recommendation_msg.reason}, linear_x={self.latest_track_recommendation_msg.linear_x:.2f}, angular_z={self.latest_track_recommendation_msg.angular_z:.2f}')
            
            # Debugging: Log the latest track follower recommendation message to see if it is valid and what the recommended linear and angular velocities are. This will help us understand how the track follower node is influencing our navigation strategy in the centreline following state.
            # self.destroy_node()
            

            if self.latest_follow_the_gap_recommendation_msg is not None and self.follow_the_gap_msg_is_fresh():
                if self.cone_nearby and self.latest_follow_the_gap_recommendation_msg.valid:
                    self.get_logger().info(
                        f'Cone detected nearby (min_front={self.min_front_distance:.2f} m), prioritising Follow The Gap'
                    )
                    self.current_state = NavigationState.FOLLOW_THE_GAP
                    return
            
            # Get the message from the track follower node and check if it is still valid based on the timestamp. If it is valid, we can use the recommended linear and angular velocities from the track follower node to control the robot's movement in the centreline following state. If the message is too old, we may choose to switch to a different navigation strategy or enter a recovery state to handle the situation appropriately.
            if self.latest_track_recommendation_msg is not None:
                self.get_logger().info(f'<DEBUG LINE 155> Latest Track Follower recommendation: valid={self.latest_track_recommendation_msg.valid}, reason={self.latest_track_recommendation_msg.reason}, linear_x={self.latest_track_recommendation_msg.linear_x:.2f}, angular_z={self.latest_track_recommendation_msg.angular_z:.2f}')

                twist_cmd_msg = Twist()
                twist_cmd_msg.linear.x = self.latest_track_recommendation_msg.linear_x
                twist_cmd_msg.angular.z = self.latest_track_recommendation_msg.angular_z
                self.publish_cmd_vel(twist_cmd_msg.linear.x, twist_cmd_msg.angular.z)
            else:
                # No track follower message yet, emit safe forward-slow movement
                self.get_logger().warn('No Track Follower message yet, publishing default slow forward motion')
                self.publish_cmd_vel(0.05, 0.0)

            pass
        elif self.current_state == NavigationState.FOLLOW_THE_GAP:
            # Implement logic for follow the gap navigation strategy
            self.get_logger().info('Current state: FOLLOW_THE_GAP')
            if self.latest_follow_the_gap_recommendation_msg is not None:
                
                # If the follow the gap recommendation indicates that the path ahead is clear, we can switch back to centreline following state to continue following the lanes on the track. This allows us to seamlessly transition between the follow the gap strategy 
                if (
                    self.latest_follow_the_gap_recommendation_msg.reason == NavigationRecommendation.NO_OBSTACLE
                    and not self.cone_nearby
                ):
                    self.get_logger().info('Follow The Gap recommendation indicates path ahead is clear, switching back to centreline following state')
                    self.current_state = NavigationState.CENTRELINE_FOLLOWING
                    return
                
                # If FTG says "yield" (NO_OBSTACLE) but cones are nearby, don't execute its zero velocities
                if (
                    self.latest_follow_the_gap_recommendation_msg.reason == NavigationRecommendation.NO_OBSTACLE
                    and self.cone_nearby
                ):
                    self.get_logger().info('FTG says clear but obstacle still near, staying in FOLLOW_THE_GAP')
                    # Keep using FTG command stream while near obstacle.
                    if self.latest_follow_the_gap_recommendation_msg.valid:
                        self.publish_cmd_vel(
                            self.latest_follow_the_gap_recommendation_msg.linear_x,
                            self.latest_follow_the_gap_recommendation_msg.angular_z
                        )
                    else:
                        self.publish_cmd_vel(0.08, 0.0)
                    return
                
                if (
                    not self.latest_follow_the_gap_recommendation_msg.valid
                    and self.latest_follow_the_gap_recommendation_msg.reason != NavigationRecommendation.NO_OBSTACLE
                ):
                    self.current_state = NavigationState.RECOVERY
                    return

                # Check the age of the follow the gap recommendation message to ensure it is still valid for making decisions in our control loop. If the message is too old, we may choose to switch to a different navigation strategy or enter a recovery state to handle the situation appropriately.
                current_time = self.get_clock().now()
                follow_the_gap_msg_age = (current_time - self.latest_follow_the_gap_recommendation_timestamp).nanoseconds / 1e9  # Convert to seconds

                if follow_the_gap_msg_age < self.max_age_time and self.latest_follow_the_gap_recommendation_msg.valid:
                    # The follow the gap recommendation message is still valid, so we can use it to control the robot's movement based on the recommended linear and angular velocities.
                    self.publish_cmd_vel(
                        self.latest_follow_the_gap_recommendation_msg.linear_x,
                        self.latest_follow_the_gap_recommendation_msg.angular_z
                    )
                else:
                    # The follow the gap recommendation message is too old or invalid, so publish safe motion
                    self.get_logger().warn('Follow the Gap recommendation message is too old or invalid, publishing safe motion')
                    self.publish_cmd_vel(0.08, 0.0)
            else:
                # No FTG message yet, emit safe forward-slow movement to keep robot moving until nodes are ready
                self.get_logger().warn('No Follow The Gap message yet, publishing default slow forward motion')
                self.publish_cmd_vel(0.05, 0.0)

            
        elif self.current_state == NavigationState.GOAL_NAVIGATION:
            # Implement logic for goal navigation strategy
            pass

        elif self.current_state == NavigationState.RECOVERY:
            self.get_logger().info('In recovery state, attempting to get unstuck or handle unexpected situation')
            
            # Move forward slowly while gently rotating to find a clear path
            self.publish_cmd_vel(0.08, 0.05)

            # Try to escape recovery if valid recommendations come back
            if self.latest_follow_the_gap_recommendation_msg is not None and self.latest_follow_the_gap_recommendation_msg.valid:
                self.get_logger().info('Follow the Gap recommendation is valid again, switching back to Follow the Gap state')
                self.current_state = NavigationState.FOLLOW_THE_GAP
                return

            if self.latest_track_recommendation_msg is not None and self.latest_track_recommendation_msg.valid:
                self.get_logger().info('Track Follower recommendation is valid again, switching back to Follow the Gap state')
                self.current_state = NavigationState.FOLLOW_THE_GAP
                return
        
        # Global safety fallback: if no fresh recommendations from either node, emit safe slow forward
        if not track_is_fresh and not ftg_is_fresh:
            self.get_logger().warn('Both navigation sources stale/unavailable, publishing fallback motion')
            self.publish_cmd_vel(0.05, 0.0)

    def track_follower_callback(self, msg):
        # This callback will be called whenever a new message is received from the track follower node. We will use this information to update our current state and make decisions about which navigation strategy to use.
        self.latest_track_recommendation_msg = msg
        self.latest_track_recommendation_timestamp = self.get_clock().now()

        self.get_logger().info(
            f'Received Track Follower cmd_vel: linear_x={msg.linear_x:.2f} m/s, angular_z={msg.angular_z:.2f} rad/s'
        )

    def follow_the_gap_callback(self, msg):
        # This callback will be called whenever a new message is received from the follow the gap node. We will use this information to update our current state and make decisions about which navigation strategy to use.
        self.latest_follow_the_gap_recommendation_msg = msg
        self.latest_follow_the_gap_recommendation_timestamp = self.get_clock().now()
        
        self.get_logger().info(
            f'Received Follow the Gap recommendation: valid={msg.valid}, confidence={msg.confidence:.2f}, linear_x={msg.linear_x:.2f}, angular_z={msg.angular_z:.2f}, reason={msg.reason}'
        )

def main():
    rclpy.init()
    behaviour_tree_node = BehaviourTree()
    rclpy.spin(behaviour_tree_node)
    behaviour_tree_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()