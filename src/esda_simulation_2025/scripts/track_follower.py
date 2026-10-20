#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
import numpy as np
import subprocess
import math

import rclpy

from rclpy.node import Node

# from build.esda_simulation_2025.rosidl_generator_py.esda_simulation_2025.msg._navigation_recommendation import NavigationRecommendation
from esda_simulation_2025.msg import NavigationRecommendation, LaneParameters


# This node is the main node that will handle the overall track following around the entire map. It will subscribe to the laser scan data and implement the follow the gap algorithm to navigate around the track. It will also handle switching to a different navigation strategy when the robot is within a certain distance to the goal.
# The pipeline will be as follows:
# 1. Get the track direction --> Uses lane detection, obstacle detection and corridors from LiDAR data to determine the direction of the track and the location of the goal.
# 2. Follow the gap algorithm to navigate around the track until the robot is within a certain distance to the goal.
# 3. Converts to motion e.g. Angular Velocity, Linear Velocity to navigate towards the goal.
# 4. Publish /cmd_vel to move the robot.

class TrackFollower(Node):
    def __init__(self):
        super().__init__('track_follower')

        # Declaring various topics and frames
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('frame_id', 'map')

        self.declare_parameter('test_track_follower_itself', False)  # Parameter to control whether to test the track follower node itself or to use the behaviour tree to manage it. This will allow us to test the track follower node in isolation without needing to run the entire behaviour tree, which can be useful for debugging and development purposes.

        # Declaring to disable to enable cmd_vel for testing
        self.declare_parameter('enable_cmd_vel', True)  # Parameter to control whether to enable publishing cmd_vel from this node. This can be useful for testing and development purposes, allowing us to disable cmd_vel publishing when we want to test the behaviour tree's decision making without actually moving the robot.
        self.enable_cmd_vel = self.get_parameter('enable_cmd_vel').get_parameter_value().bool_value

        # Declaring gains
        self.declare_parameter('K_steering', 1.0)  # Gain for the steering control, this can be adjusted based on the desired responsiveness of the steering control. A higher gain will result in more aggressive steering, while a lower gain will result in smoother but less responsive steering.
        self.declare_parameter('K_speed', 0.5)  # Gain for the speed control, this can be adjusted based on the desired responsiveness of the speed control. A higher gain will result in more aggressive speed changes, while a lower gain will result in smoother but less responsive speed control.
        self.K_steering = self.get_parameter('K_steering').get_parameter_value().double_value
        self.K_speed = self.get_parameter('K_speed').get_parameter_value().double_value


        self.test_track_follower_itself = self.get_parameter('test_track_follower_itself').get_parameter_value().bool_value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) 

        # Publisher for lane parameters to assist in determining the track direction and to assist in switching between different navigation strategies based on the distance to the goal. This will allow us to publish the computed lane parameters (e.g. lane curvature, lane offset, etc.) to a topic that can be subscribed to by the behaviour tree or other nodes to assist in decision making for navigation.
        self.lane_parameters_publisher = self.create_publisher(LaneParameters, '/lane_parameters', 10)  

        self.goal_distance_threshold = 0.5  # Distance threshold to switch to goal navigation
        self.current_goal = None  # Placeholder for the current goal position

        
        self.track_width = 1.7 # Assume a 4 metre track width for computing the centreline from the lane detection data, this can be adjusted based on the actual track width in the simulation or real world environment.
        self.one_lane_threshold_factor = 1.2  # Threshold for determining if we only see one lane, this can be adjusted based on the expected distance between the lanes and the noise in the lane detection data. If the average x position of the detected lane points is within this threshold from the desired offset, we can consider that we only see one lane and adjust our control strategy accordingly.

        self.temp_lane_timer_cb_test = False  # Temporary variable to control whether to run the lane_timer_callback for testing purposes, this can be removed once we have the lane_timer_callback fully implemented and tested.
    
        self.last_angular_z = 0.0  # Variable to store the last angular velocity command, this can be used to implement smoothing or rate limiting of the angular velocity commands if needed. Used for when we are unclear of the lane evidence, we can choose to maintain our last angular velocity command to try to keep a consistent heading rather than making sudden changes in direction based on uncertain lane evidence.

        self.left_lane = []
        self.right_lane = []

        # Subscribes to LiDAR laser scan data
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            self.get_parameter('lidar_topic').get_parameter_value().string_value, 
            self.lidar_listener_callback, 
            10
        )

        # Subscribes to lane detection data (if available) to assist in determining track direction
        self.lane_subscriber = self.create_subscription(
            MarkerArray, 
            '/lane_markers', 
            self.lane_listener_callback, 
            10
        )

        # self.behaviour_tree_subscriber = self.create_subscription(
        #     Twist, 
        #     '/cmd_vel', 
        #     self.track_follower_callback, 
        #     10
        # )

        self.behaviour_tree_publisher = self.create_publisher(
            NavigationRecommendation,
            '/track_follower_recommendation', 
            10
        )

        # Creates a timer to periodically compute the centreline and desired heading based on the latest LiDAR and lane detection data
        self.timer = self.create_timer(0.1, self.lane_timer_callback)

        print("Track Follower node initialized and subscribed to LiDAR and lane detection topics.")

    def valid_lane_pair(self, left_eq, right_eq, lookahead_z=1.5):
        if left_eq is None or right_eq is None:
            return False

        left_x = left_eq[0] * lookahead_z + left_eq[1]
        right_x = right_eq[0] * lookahead_z + right_eq[1]

        measured_width = abs(right_x - left_x)

        # Left lane should be left of robot, right lane should be right of robot
        if left_x > -0.15:
            return False

        if right_x < 0.15:
            return False

        # Lane pair should have reasonable separation
        if measured_width < 0.8:
            return False
        
        # Reject both if thee gradients have the wrong signs (e.g. both positive or both negative), as this is unlikely to be a valid lane pair and may indicate that we are seeing two fragments of the same lane rather than two separate lanes. By checking the signs of the gradients of the left and right lane equations, we can filter out cases where both lanes have the same sign, which is unlikely to be a valid lane pair and may indicate that we are seeing two fragments of the same lane rather than two separate lanes. This can help us improve the robustness of our lane detection and tracking by ensuring that we are only considering valid lane pairs that are likely to represent the actual left and right lanes of the track.
        if not (left_eq[0] < 0.0 and right_eq[0] > 0.0):
            return False

        return True

    def get_line_lane_equation(self, lane_points):
        # Implement logic to compute the line equation for a lane based on the detected lane points. This can be used to assist in determining the track direction and to assist in switching between different navigation strategies based on the distance to the goal.
        # Getting the line equation for the left and right lanes can help us determine the track direction and can also assist in switching between different navigation strategies based on the distance to the goal. For example, if we are far from the goal, we may want to follow the lane more closely, while if we are close to the goal, we may want to switch to a different navigation strategy that focuses more on navigating towards the goal rather than following the lane.
        if lane_points is None or len(lane_points) < 2:
            return None  # Not enough points to compute a line equation
        
        # Extract x and z coordinates from the lane points
        z_values = np.array([point[2] for point in lane_points], dtype=float)
        x_values = np.array([point[0] for point in lane_points], dtype=float)

        # Fit a line of the form x = m*z + c to the lane points using numpy's polyfit function, treating z as the independent variable and x as the dependent variable. This will give us the slope (m) and intercept (c) of the line equation, which we can use to determine the track direction and to assist in switching between different navigation strategies based on the distance to the goal.
        m, c = np.polyfit(z_values, x_values, 1)  # Fit a line to the lane points, treating z as the independent variable and x as the dependent variable. This will give us the slope (m) and intercept (c) of the line equation in the form x = m*z + c.
        return m, c 
    
    def check_lines_sign(self, left_line_eq, right_line_eq):
        # Implement logic to check signs of left and right line equations to determine track direction. This can be used to assist in determining the track direction and to assist in switching between different navigation strategies based on the distance to the goal.
        # By checking the signs of the slopes of the left and right line equations, we can determine the track direction. For example, if the slope of the left lane line is positive and the

        # Right line should have a positive gradient
        # Left line should have a negative gradient
        if left_line_eq is None or right_line_eq is None:
            return None  # Cannot determine track direction without both line equations
        
        left_slope = left_line_eq[0]
        right_slope = right_line_eq[0]

        if left_slope < 0 and right_slope > 0:
            return 'forward'
        elif left_slope > 0 and right_slope < 0:
            return 'backward'
        elif left_slope < 0 and right_slope < 0:
            # Move heading towards the left lane to try to get a better view of the track direction
            return 'left'
        elif left_slope > 0 and right_slope > 0:
            # Move heading towards the right lane to try to get a better view of the track direction
            return 'right'
        
    def check_measured_lane_width(self, left_line_eq, right_line_eq, lookahead_z=1.5):
        # IMPORTANT: SINCE THIS TAKES INTO ACCOUNT IMPERFECT LANES
        # Implement logic to check the measured lane width based on the left and right line equations at a certain lookahead distance (lookahead_z) to determine if it is consistent with the expected track width. This can be used to assist in determining the track direction and to assist in switching between different navigation strategies based on the distance to the goal.
        if left_line_eq is None or right_line_eq is None:
            return None  # Cannot check lane width without both line equations
        
        left_x = left_line_eq[0] * lookahead_z + left_line_eq[1]
        right_x = right_line_eq[0] * lookahead_z + right_line_eq[1]

        measured_lane_width = abs(right_x - left_x)

        self.get_logger().info(f"\nMeasured lane width at lookahead distance {lookahead_z}: {measured_lane_width:.2f} (Left x: {left_x:.2f}, Right x: {right_x:.2f})\n")

        centre_x = (left_x + right_x) / 2.0
        desired_centre_x = 0.0  # Assuming we want to be in the centre of the track, which is at x = 0.0 in our coordinate system. This

        if measured_lane_width < self.track_width * 0.6:
            width_status = 'too_narrow'
        elif measured_lane_width > self.track_width * 1.3:
            width_status = 'too_wide'
        else:
            width_status = 'normal'

        if centre_x > 0.2:
            direction = 'lanes_shifted_right'
        elif centre_x < -0.2:
            direction = 'lanes_shifted_left'
        else:
            direction = 'lanes_centered'

        return width_status, direction

    def classify_lane_evidence_from_points(self, left_eq, right_eq, lookahead_z=1.5):
        if left_eq is None and right_eq is None:
            return "no_lanes", None

        if left_eq is not None and right_eq is not None:
            left_x = left_eq[0] * lookahead_z + left_eq[1]
            right_x = right_eq[0] * lookahead_z + right_eq[1]
            width = abs(right_x - left_x)

            self.get_logger().info(
                f"lane evidence: left_x={left_x:.2f}, right_x={right_x:.2f}, width={width:.2f}"
            )

            # Real usable lane pair
            if width >= 0.8 and left_x < -0.15 and right_x > 0.15 and left_eq[0] < 0.0 and right_eq[0] > 0.0:
                return "valid_pair", "lane_evidence_center"

            # Too narrow = probably same-side fragments
            if width < 0.8:
                if right_x > 0.15:
                    return "invalid_or_single_side", "lane_evidence_right"
                elif left_x < -0.15:
                    return "invalid_or_single_side", "lane_evidence_left"
                else:
                    return "invalid_or_single_side", "lane_evidence_unclear"

            # Both lines on same side
            if left_x > 0.15 and right_x > 0.15:
                return "invalid_or_single_side", "lane_evidence_right"
            elif left_x < -0.15 and right_x < -0.15:
                return "invalid_or_single_side", "lane_evidence_left"

            return "invalid_or_single_side", "lane_evidence_unclear"

        # Only one lane equation exists
        if left_eq is not None:
            x = left_eq[0] * lookahead_z + left_eq[1]
        else:
            x = right_eq[0] * lookahead_z + right_eq[1]

        if x > 0.15:
            return "invalid_or_single_side", "lane_evidence_right"
        elif x < -0.15:
            return "invalid_or_single_side", "lane_evidence_left"
        else:
            return "invalid_or_single_side", "lane_evidence_unclear"

    

    def lane_timer_callback(self):
        # This timer callback will be called periodically to compute the centreline and desired heading based on the latest LiDAR and lane detection data. It will then publish the appropriate cmd_vel to navigate towards the goal.
        lookahead_z = 1.5

        left_eq = self.get_line_lane_equation(self.left_lane)
        right_eq = self.get_line_lane_equation(self.right_lane)

        self.get_logger().info(
            f"Running lane_timer_callback. left_eq={left_eq}, right_eq={right_eq}, "
            f"check_lines_sign={self.check_lines_sign(left_eq, right_eq)}"
        )

        lane_msg = LaneParameters()
        lane_msg.left_lane_gradient = left_eq[0] if left_eq else 0.0
        lane_msg.left_lane_x_intercept = left_eq[1] if left_eq else 0.0
        lane_msg.right_lane_gradient = right_eq[0] if right_eq else 0.0
        lane_msg.right_lane_x_intercept = right_eq[1] if right_eq else 0.0
        lane_msg.confidence = 1.0 if left_eq and right_eq else 0.5
        self.lane_parameters_publisher.publish(lane_msg)

        rec = NavigationRecommendation()
        rec.source = NavigationRecommendation.TRACK_FOLLOWER
        rec.valid = True
        rec.confidence = lane_msg.confidence

        # Case 1: both lanes visible
        if left_eq is not None and right_eq is not None:

            lane_pair_status, lane_evidence_side = self.classify_lane_evidence_from_points(
                left_eq,
                right_eq,
                lookahead_z
            )

            self.get_logger().info(
                f"lane_pair_status={lane_pair_status}, "
                f"lane_evidence_side={lane_evidence_side}"
            )

            

            if lane_pair_status != "valid_pair":

                _, lane_evidence_side = self.classify_lane_evidence_from_points(left_eq, right_eq, lookahead_z)

                rec.reason = NavigationRecommendation.INVALID_LANE_PAIR 
                reason = f"{lane_pair_status}_{lane_evidence_side}"
                rec.valid = True
                rec.confidence = 0.3
                rec.linear_x = 0.08

                if lane_evidence_side == "lane_evidence_right":
                    rec.angular_z = 0.7   # steer left away from right line
                    self.last_angular_z = rec.angular_z  # Update last angular velocity command when we have some lane evidence to steer away from right line
                elif lane_evidence_side == "lane_evidence_left":
                    rec.angular_z = -0.7  # steer right away from left line
                    self.last_angular_z = rec.angular_z  # Update last angular velocity command when we have some lane evidence to steer away from left line
                else: # When lane_evidence_side = "lane_evidence_unclear", we will just steer based on the centre error but with reduced confidence and speed since we are not sure about the lane evidence
                    # rec.linear_x = 0.05
                    # rec.angular_z = 0.5 * self.last_angular_z  # maintain last angular velocity command to try to keep a consistent heading rather than making sudden changes in direction based on uncertain lane evidence
                    rec.angular_z = 0.0  # steer based on centre error but with reduced gain since we are not sure about the lane evidence

                self.behaviour_tree_publisher.publish(rec)
                return

            width_status, lane_direction = self.check_measured_lane_width(
                left_eq, right_eq, lookahead_z
            )

            left_x = left_eq[0] * lookahead_z + left_eq[1]
            right_x = right_eq[0] * lookahead_z + right_eq[1]
            centre_x = (left_x + right_x) / 2.0

            rec.linear_x = 0.18

            # IMPORTANT:
            # centre_x < 0 means lane centre is left of robot,
            # so turn left: positive angular_z.
            centre_error = -centre_x

            if width_status == "normal":
                rec.reason = NavigationRecommendation.BOTH_LANES_NORMAL_LANES_CENTRED
                reason = f"both_lanes_normal_{lane_direction}"
                rec.angular_z = max(min(0.6 * centre_error, 0.3), -0.3)

            elif width_status == "too_narrow":
                rec.reason = NavigationRecommendation.BOTH_LANES_TOO_NARROW_CENTRED
                reason = f"both_lanes_too_narrow_{lane_direction}"
                rec.linear_x = 0.10

                if lane_direction == "lanes_shifted_left":
                    rec.angular_z = 0.18      # steer left
                elif lane_direction == "lanes_shifted_right":
                    rec.angular_z = -0.18     # steer right
                else:
                    rec.angular_z = max(min(0.4 * centre_error, 0.2), -0.2)

            elif width_status == "too_wide":
                rec.reason = NavigationRecommendation.BOTH_LANES_TOO_WIDE_CENTRED
                reason = f"both_lanes_too_wide_{lane_direction}"
                rec.confidence = 0.4
                rec.linear_x = 0.08
                rec.angular_z = max(min(0.4 * centre_error, 0.2), -0.2)

        # Case 2: only left lane visible
        elif left_eq is not None:
            left_x = left_eq[0] * lookahead_z + left_eq[1]

            desired_left_x = -self.track_width / 2.0
            error = desired_left_x - left_x

            rec.reason = NavigationRecommendation.ONLY_LEFT_LANE_VISIBLE
            reason = "only_left_lane_visible"
            rec.linear_x = 0.12
            rec.angular_z = max(min(-0.4 * error, 0.25), -0.25)

        # Case 3: only right lane visible
        elif right_eq is not None:
            right_x = right_eq[0] * lookahead_z + right_eq[1]

            desired_right_x = self.track_width / 2.0
            error = desired_right_x - right_x

            rec.reason = NavigationRecommendation.ONLY_RIGHT_LANE_VISIBLE
            reason = "only_right_lane_visible"
            rec.linear_x = 0.12
            rec.angular_z = max(min(-0.4 * error, 0.25), -0.25)

        # Case 4: no lanes visible
        else:
            rec.reason = NavigationRecommendation.NO_LANES_VISIBLE
            reason = "no_lanes_visible"
            rec.valid = False
            rec.confidence = 0.0
            rec.linear_x = 0.0
            rec.angular_z = 0.0

        self.get_logger().info(
            f"Track follower rec: reason={reason}, "
            f"linear_x={rec.linear_x:.2f}, angular_z={rec.angular_z:.2f}, "
            f"confidence={rec.confidence:.2f}, valid={rec.valid}"
        )

        self.behaviour_tree_publisher.publish(rec)


    def lidar_listener_callback(self, msg):
        # Access LiDAR data from the LaserScan message
        ranges = msg.ranges

    def lane_listener_callback(self, msg):
        # Access lane detection data from the MarkerArray message
        markers = msg.markers

        left_points = []
        right_points = []

        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z

            point = (x, y, z)

            if marker.ns == 'left_lane':
                left_points.append(point)
            elif marker.ns == 'right_lane':
                right_points.append(point)
        
        self.left_lane = left_points
        self.right_lane = right_points

        left_line_equation = self.get_line_lane_equation(self.left_lane)
        right_line_equation = self.get_line_lane_equation(self.right_lane)

        print(f"len(self.left_lane): {len(self.left_lane)}. Left Lane Equation: x = {left_line_equation[0]} * z + {left_line_equation[1]}" if left_line_equation else "No left lane points to compute line equation.")
        print(f"len(self.right_lane): {len(self.right_lane)}. Right Lane Equation: x = {right_line_equation[0]} * z + {right_line_equation[1]}" if right_line_equation else "No right lane points to compute line equation.")

    def compute_centreline(self):
        # Can't compute a centreline if we don't have both left and right lane points
        if not self.left_lane or not self.right_lane:
            return []

        # Sort the lane points by their z-coordinate (distance along the track) to ensure we are matching points that are at similar positions along the track
        left_sorted = sorted(self.left_lane, key=lambda p: p[2])
        right_sorted = sorted(self.right_lane, key=lambda p: p[2])

        # Determine which lane has fewer points to use as the primary for matching, to ensure we are not trying to match more points than we have in the other lane
        if len(left_sorted) <= len(right_sorted):
            primary = left_sorted
            secondary = right_sorted
            primary_is_left = True
        else:
            primary = right_sorted
            secondary = left_sorted
            primary_is_left = False

        # Match points from the primary lane to the closest point in the secondary lane based on their z-coordinate (distance along the track) to compute the centreline points. We will only consider matches that are within a certain z-distance threshold to ensure we are matching points that are close enough along the track.
        centreline = []
        max_z_diff = 0.5

        # For each point in the primary lane, find the closest point in the secondary lane based on their z-coordinate (distance along the track) and compute the centreline point as the midpoint between the two matched points. We will only consider matches that are within a certain z-distance threshold to ensure we are matching points that are close enough along the track.
        for px, py, pz in primary:
            best_match = None
            best_diff = float('inf')

            # Iterate through the secondary lane points to find the closest point based on z-coordinate (distance along the track)
            for sx, sy, sz in secondary:
                diff = abs(sz - pz)
                if diff < best_diff:
                    best_diff = diff
                    best_match = (sx, sy, sz)

            # Only consider matches that are within the z-distance threshold to ensure we are matching points that are close enough along the track
            if best_match is None or best_diff > max_z_diff:
                continue

            sx, sy, sz = best_match

            if primary_is_left:
                lx, ly, lz = px, py, pz
                rx, ry, rz = sx, sy, sz
            else:
                lx, ly, lz = sx, sy, sz
                rx, ry, rz = px, py, pz

            centreline.append((
                (lx + rx) / 2.0,
                (ly + ry) / 2.0,
                (lz + rz) / 2.0
            ))

        return centreline
    
    # TODO: Implement the logic to follow the computed centreline
    def follow_track(self):
        # This method sees the best continuous lane and tries to maintain / confine the robot towards it
        left_lane_points = self.left_lane
        right_lane_points = self.right_lane

def main(args=None):
    rclpy.init(args=args)
    track_follower = TrackFollower()
    rclpy.spin(track_follower)
    track_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()