#!/usr/bin/env python3

from enum import Enum
import math
import numpy as np

import xml.etree.ElementTree as ET



import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist 
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs import msg
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan

from itertools import groupby

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import subprocess

# from build.esda_simulation_2025.rosidl_generator_py.esda_simulation_2025.msg._navigation_recommendation import NavigationRecommendation
from esda_simulation_2025.msg import NavigationRecommendation, LaneParameters


# Define the FollowTheGap class, which will implement the "Follow the Gap" algorithm for navigation
# This algorithm does not seek out the longest gap, but rather the gap that is closest to the goal direction, which can be more efficient in certain scenarios.
# Approach uses the Disparity Extender method to identify gaps in the occupancy grid and navigate through them towards the goal.
# The algorithm will be implemented as a ROS2 node that subscribes to the occupancy grid and publishes navigation goals to the Nav2 stack.
# Dispariities are gaps in the LiDAR scan data where 2 numbers next to each other differ majorly, indicating a potential gap in the environment. The algorithm will identify these gaps and evaluate them based on their alignment with the goal direction to determine the best path forward.

# 1. Find disparities in lidar readings
# 2. For each disparity, extend it half the width of the robot to find the gap
# 3. Evaluate the gap based on its alignment with the goal direction
# 4. Select the best gap and navigate towards it

# Making sure that car doesn't hit a corner
# 1. Scan all available LiDAR samples below -90 degrees and above 90 degrees
# 2. If any of these samples are below a certain threshold, consider the path blocked... or if any point is below safe distance on side of car in the direction the car is going, stop turning and keep going straight
# 3. If the path is blocked, stop the robot and re-evaluate the environment

# Wigglling problem - Robot keeps turning left and right --> 'S' shape --> Set limit threshold e,g, 2m or 3m, then follow centre of deepest gap until the robot is within the threshold distance to the goal, then switch to a different navigation strategy (e.g., A* or Dijkstra's) to navigate the remaining distance to the goal.

# NOTE: This code is to be used in conjunction with track_follower.py. which will handle the overall track following around the entire map
# NOTE: Follow the Gap must also be used before the track follower as the track follower seems to detect the cones as a lane...

class States(Enum):
    NO_SAFE_GAPS = 0
    GAP_FOLLOWING = 1
    PATH_AHEAD_CLEAR = 2

def load_robot_xml(filepath):
    result = subprocess.run(
        ['xacro', filepath],
        capture_output=True,
        text=True
    )
    return result.stdout

def extract_param_from_xml(xml_text, property_name='chassis_width'):
    root = ET.fromstring(xml_text)

    # ---- 1. Try to find property (ONLY works if xacro NOT expanded) ----
    for elem in root.iter():
        tag = elem.tag.split('}')[-1]

        if tag == 'property' and elem.attrib.get('name') == property_name:
            try:
                return float(elem.attrib.get('value'))
            except (TypeError, ValueError):
                pass

    # ---- 2. Fallback: extract from geometry (expanded XML) ----
    for elem in root.iter():
        tag = elem.tag.split('}')[-1]

        if tag == 'box':
            size = elem.attrib.get('size', '')
            parts = size.split()

            if len(parts) == 3:
                try:
                    # x, y, z → width is y
                    return float(parts[1])
                except ValueError:
                    pass

    return None

def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')

        package_share = Path(get_package_share_directory('esda_simulation_2025'))
        robot_description_file = package_share / 'description' / 'robot_core_ref.xacro'

        self.declare_parameter('robot_description_file', str(robot_description_file))
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('frame_id', 'map')

        # FTG tuning
        self.declare_parameter('disparity_threshold', 0.5)
        self.declare_parameter('safe_distance', 1.5)
        self.declare_parameter('safety_margin', 0.10)
        self.declare_parameter('max_speed', 0.45)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('max_turn_rate', 1.5)
        self.declare_parameter('steering_gain', 0.8)
        self.declare_parameter('forward_bias_gain', 0.35)
        
        self.declare_parameter('obstacle_risk_distance', 1.6)
        self.declare_parameter('gap_depth_weight_base', 0.8)
        self.declare_parameter('gap_depth_weight_gain', 1.0)
        self.declare_parameter('gap_width_weight_base', 0.015)
        self.declare_parameter('gap_width_weight_gain', 0.02)
        self.declare_parameter('angle_penalty_reduction', 0.7)
        self.declare_parameter('angle_penalty_floor_scale', 0.35)
        self.declare_parameter('min_ftg_confidence', 0.45)
        self.declare_parameter('max_ftg_confidence', 1.0)
        self.declare_parameter('lane_constraint_enabled', True)
        self.declare_parameter('lane_projection_distance', 1.2)
        self.declare_parameter('lane_boundary_margin', 0.12)
        self.declare_parameter('lane_center_penalty_gain', 0.8)
        self.declare_parameter('recommendation_config', True) # Whether to publish recommendations to the behaviour tree node, can be set to False for testing the FTG algorithm in isolation without affecting the overall behaviour tree logic. This allows for more focused testing and debugging of the FTG algorithm itself or straight /cmd_vel commands, without needing to consider the interactions with the behaviour tree node

        # Weighting parameters
        self.ftg_confidence_weight = 0.0
        

        self.recommendation_config = self.get_parameter('recommendation_config').get_parameter_value().bool_value

        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.disparity_threshold = self.get_parameter('disparity_threshold').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_turn_rate = self.get_parameter('max_turn_rate').value
        self.steering_gain = self.get_parameter('steering_gain').value
        self.forward_bias_gain = self.get_parameter('forward_bias_gain').value
        self.obstacle_risk_distance = self.get_parameter('obstacle_risk_distance').value
        self.gap_depth_weight_base = self.get_parameter('gap_depth_weight_base').value
        self.gap_depth_weight_gain = self.get_parameter('gap_depth_weight_gain').value
        self.gap_width_weight_base = self.get_parameter('gap_width_weight_base').value
        self.gap_width_weight_gain = self.get_parameter('gap_width_weight_gain').value
        self.angle_penalty_reduction = self.get_parameter('angle_penalty_reduction').value
        self.angle_penalty_floor_scale = self.get_parameter('angle_penalty_floor_scale').value
        self.min_ftg_confidence = self.get_parameter('min_ftg_confidence').value
        self.max_ftg_confidence = self.get_parameter('max_ftg_confidence').value
        self.lane_constraint_enabled = self.get_parameter('lane_constraint_enabled').value
        self.lane_projection_distance = self.get_parameter('lane_projection_distance').value
        self.lane_boundary_margin = self.get_parameter('lane_boundary_margin').value
        self.lane_center_penalty_gain = self.get_parameter('lane_center_penalty_gain').value

        robot_file = self.get_parameter('robot_description_file').get_parameter_value().string_value
        robot_xml = load_robot_xml(robot_file)

        self.robot_x_width = extract_param_from_xml(robot_xml, 'chassis_width')
        self.robot_y_width = extract_param_from_xml(robot_xml, 'chassis_length')

        if self.robot_x_width is None:
            self.robot_x_width = 0.5
            self.get_logger().warn('Could not read chassis_width from xacro, defaulting to 0.5 m')

        if self.robot_y_width is None:
            self.robot_y_width = 0.7
            self.get_logger().warn('Could not read chassis_length from xacro, defaulting to 0.7 m')

        self.robot_radius = self.robot_x_width / 2.0
        self.ftg_safety_radius = self.robot_radius + self.safety_margin

        self.latest_map = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_lane_detected = False
        self.right_lane_detected = False
        self.left_lane_gradient = 0.0
        self.left_lane_x_intercept = 0.0
        self.right_lane_gradient = 0.0
        self.right_lane_x_intercept = 0.0
        self.lane_confidence = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.forward_distance_threshold = 1.0  # Minimum distance to consider a gap safe in front of the robot. This can be tuned based on the robot's speed and stopping distance to ensure that the robot has enough time to react to obstacles in front of it while following gaps.
        self.forward_scan_angle = math.radians(20.0)  # Angle range for checking the path ahead, e.g., 20 degrees to either side of the forward direction. This can be tuned to balance between being too cautious (narrow angle) and missing obstacles (wide angle) when evaluating if the path ahead is clear.

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.scan_callback,
            10
        )

        self.occupancy_grid_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.occupancy_grid_callback,
            10
        )

        self.behaviour_tree_publisher = self.create_publisher(
            NavigationRecommendation,
            '/follow_the_gap_recommendation', 
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to lane parameters from the track follower node to ensure that the robot stays within the lane while following gaps. This can help improve safety and reliability by leveraging the lane information provided by the track follower node to avoid navigating towards gaps that lead outside of the lane boundaries.
        self.lane_parameters_subscriber = self.create_subscription(
            LaneParameters,
            '/lane_parameters',
            self.lane_parameters_callback,
            10
        )

        # State variables for debugging and potential future use in the behaviour tree node
        self.previous_state = None
        self.current_state = None

        self.get_logger().info('FollowTheGap cmd_vel node initialised')
        self.get_logger().info(f'Robot width: {self.robot_x_width:.3f} m')
        self.get_logger().info(f'Robot length: {self.robot_y_width:.3f} m')
        self.get_logger().info(f'Robot radius: {self.robot_radius:.3f} m')
        self.get_logger().info(f'FTG safety radius: {self.ftg_safety_radius:.3f} m')
        self.get_logger().info(f'Safe distance threshold: {self.safe_distance:.3f} m')

    def is_within_lane(self, right_lane_parameters, left_lane_parameters, left_lane_detected=False, right_lane_detected=False, lane_confidence=0.0):
        # Used to confine the robot's navigation within the lane boundaries by checking if the robot's current position is within the lane defined by the right and left lane parameters. This can help improve safety and reliability by avoiding navigation towards gaps that lead outside of the lane boundaries.
        self.right_lane_gradient = float(right_lane_parameters[0])
        self.right_lane_x_intercept = float(right_lane_parameters[1])
        self.left_lane_gradient = float(left_lane_parameters[0])
        self.left_lane_x_intercept = float(left_lane_parameters[1])
        self.left_lane_detected = bool(left_lane_detected)
        self.right_lane_detected = bool(right_lane_detected)
        self.lane_confidence = float(np.clip(lane_confidence, 0.0, 1.0))

    def lane_lateral_at_forward_distance(self, lane_gradient, lane_intercept, forward_distance):
        return lane_gradient * forward_distance + lane_intercept

    def lane_confinement_check(self, gap_center_angle, gap_depth):
        if not self.lane_constraint_enabled:
            return True, 0.0

        if not (self.left_lane_detected or self.right_lane_detected):
            return True, 0.0

        projection_distance = max(0.25, min(float(gap_depth), self.lane_projection_distance))
        gap_lateral = projection_distance * math.sin(gap_center_angle)

        lane_penalty = 0.0

        if self.left_lane_detected:
            left_bound = self.lane_lateral_at_forward_distance(
                self.left_lane_gradient,
                self.left_lane_x_intercept,
                projection_distance
            )
            if gap_lateral > (left_bound - self.lane_boundary_margin):
                return False, -1.0
            lane_penalty += self.lane_center_penalty_gain * math.exp(-(left_bound - gap_lateral))

        if self.right_lane_detected:
            right_bound = self.lane_lateral_at_forward_distance(
                self.right_lane_gradient,
                self.right_lane_x_intercept,
                projection_distance
            )
            if gap_lateral < (right_bound + self.lane_boundary_margin):
                return False, -1.0
            lane_penalty += self.lane_center_penalty_gain * math.exp(-(gap_lateral - right_bound))

        if self.left_lane_detected and self.right_lane_detected:
            center_line = 0.5 * (left_bound + right_bound)
            half_width = max(0.05, 0.5 * (left_bound - right_bound))
            center_offset = abs(gap_lateral - center_line) / half_width
            lane_penalty += self.lane_center_penalty_gain * np.clip(center_offset, 0.0, 2.0)

        confidence_scale = 0.3 + 0.7 * self.lane_confidence
        return True, -confidence_scale * lane_penalty
    
    def lane_parameters_callback(self, msg):
        # This callback can be used to receive lane parameters from the track follower node, which can then be used to enhance the gap scoring by considering the lane information. For example, if the lane parameters indicate that the robot is close to the edge of the lane, the gap scoring can be adjusted to prefer gaps that are more centered within the lane, or to avoid gaps that lead towards the edge of the lane. This can help improve the safety and reliability of the navigation by leveraging the lane information provided by the track follower node.
        right_lane_gradient = msg.right_lane_gradient
        right_lane_x_intercept = msg.right_lane_x_intercept
        left_lane_gradient = msg.left_lane_gradient
        left_lane_x_intercept = msg.left_lane_x_intercept
        left_lane_detected = msg.left_lane_detected
        right_lane_detected = msg.right_lane_detected
        lane_confidence = msg.confidence
        
        self.is_within_lane(
            right_lane_parameters=(right_lane_gradient, right_lane_x_intercept),
            left_lane_parameters=(left_lane_gradient, left_lane_x_intercept),
            left_lane_detected=left_lane_detected,
            right_lane_detected=right_lane_detected,
            lane_confidence=lane_confidence
        )

    def footprint_is_map_safe(self, yaw, max_distance=1.5, step=0.10):
        if self.latest_map is None:
            return True

        # half_width = self.robot_x_width / 2.0
        # offsets = [-half_width, 0.0, half_width]
        half_width = 0.4 * self.robot_x_width
        offsets = [-half_width, 0.0, half_width]

        d = step
        while d <= max_distance:
            cx = self.x + d * math.cos(yaw)
            cy = self.y + d * math.sin(yaw)

            # lateral vector
            lx = -math.sin(yaw)
            ly = math.cos(yaw)

            for offset in offsets:
                wx = cx + offset * lx
                wy = cy + offset * ly

                value = self.get_map_value(wx, wy)

                if value is None or value >= 80:
                    return False

            d += step

        return True

    def gap_is_map_safe(self, yaw, max_distance=1.5, step=0.10):
        if self.latest_map is None:
            return True

        d = step
        while d <= max_distance:
            wx = self.x + d * math.cos(yaw)
            wy = self.y + d * math.sin(yaw)

            value = self.get_map_value(wx, wy)

            if value is None:
                return False
            if value >= 50:
                return False

            d += step

        return True

    def world_to_map(self, wx, wy):
        if self.latest_map is None:
            return None

        info = self.latest_map.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None

        return mx, my

    def map_index(self, mx, my):
        return my * self.latest_map.info.width + mx

    def get_map_value(self, wx, wy):
        cell = self.world_to_map(wx, wy)
        if cell is None:
            return None

        mx, my = cell
        return self.latest_map.data[self.map_index(mx, my)]

    def score_map_ray(self, yaw, max_distance=2.0, step=0.15):
        if self.latest_map is None:
            return 0.0

        score = 0.0
        d = step

        while d <= max_distance:
            wx = self.x + d * math.cos(yaw)
            wy = self.y + d * math.sin(yaw)

            value = self.get_map_value(wx, wy)

            if value is None:
                score -= 5.0
                break
            elif value >= 50:
                score -= 8.0
                break
            elif value == -1:
                score -= 1.0
            else:
                score += 0.5

            d += step

        return score

    def compute_obstacle_risk(self, forward_ranges):
        if forward_ranges.size == 0:
            return 0.0

        risk_distance = max(self.safe_distance, self.obstacle_risk_distance)
        near_ratio = float(np.mean(forward_ranges < risk_distance))
        very_near_ratio = float(np.mean(forward_ranges < (0.75 * risk_distance)))

        min_forward = float(np.min(forward_ranges))
        proximity = np.clip((risk_distance - min_forward) / risk_distance, 0.0, 1.0)

        obstacle_risk = np.clip(
            0.5 * near_ratio + 0.3 * very_near_ratio + 0.2 * proximity,
            0.0,
            1.0
        )

        return float(obstacle_risk)
        
    def occupancy_grid_callback(self, msg):
        # Process occupancy grid data here if needed for the follow the gap algorithm, e.g., to evaluate gaps based on the occupancy grid information. This can be used to enhance the gap scoring by considering not only the LiDAR data but also the known map of the environment.
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_data = np.array(msg.data, dtype=np.int16).reshape((self.map_height, self.map_width))
        self.latest_map = msg

    def publish_cmd(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)
        
        follow_the_gap_msg_recommendation = NavigationRecommendation()
        follow_the_gap_msg_recommendation.source = NavigationRecommendation.FOLLOW_THE_GAP 
        follow_the_gap_msg_recommendation.valid = False
        follow_the_gap_msg_recommendation.confidence = 0.0
        follow_the_gap_msg_recommendation.linear_x = 0.0
        follow_the_gap_msg_recommendation.angular_z = 0.0
        follow_the_gap_msg_recommendation.reason = NavigationRecommendation.RECOVERY_REQUIRED

        self.behaviour_tree_publisher.publish(follow_the_gap_msg_recommendation)

    def find_disparities(self, lidar_data):
        disparities = []

        if len(lidar_data) < 2:
            return disparities

        for i in range(1, len(lidar_data)):
            left = float(lidar_data[i - 1])
            right = float(lidar_data[i])

            if abs(right - left) > self.disparity_threshold:
                disparities.append({
                    'left_index': i - 1,
                    'right_index': i,
                    'left_distance': left,
                    'right_distance': right,
                    'closer_index': i - 1 if left < right else i,
                    'closer_distance': min(left, right),
                })

        return disparities

    def extend_disparity(self, forward_ranges, disparities, angle_increment):
        extended_ranges = np.array(forward_ranges, copy=True)

        if len(extended_ranges) == 0 or len(disparities) == 0:
            return extended_ranges

        for disparity in disparities:
            closer_index = disparity['closer_index']
            closer_distance = disparity['closer_distance']
            left_distance = disparity['left_distance']
            right_distance = disparity['right_distance']

            if closer_distance <= 0.0 or angle_increment <= 0.0:
                continue

            points_to_extend = int(
                math.ceil((self.ftg_safety_radius / closer_distance) / angle_increment)
            )

            if left_distance < right_distance:
                start = max(0, closer_index - points_to_extend)
                end = closer_index + 1
            else:
                start = closer_index
                end = min(len(extended_ranges), closer_index + points_to_extend + 1)

            extended_ranges[start:end] = np.minimum(
                extended_ranges[start:end],
                closer_distance
            )

        return extended_ranges

    def score_gap(self, extended_ranges, safe_groups, forward_angles, obstacle_risk=0.0):
        best_score = -float('inf')
        best_gap = None

        depth_weight = self.gap_depth_weight_base + self.gap_depth_weight_gain * obstacle_risk
        width_weight = self.gap_width_weight_base + self.gap_width_weight_gain * obstacle_risk
        angle_weight = self.forward_bias_gain * max(
            self.angle_penalty_floor_scale,
            1.0 - self.angle_penalty_reduction * obstacle_risk
        )

        for is_safe, indices in safe_groups:
            if not is_safe or len(indices) == 0:
                continue

            gap_angles = forward_angles[indices]
            gap_center_angle = float(np.mean(gap_angles))
            gap_depth = float(np.mean(extended_ranges[indices]))
            gap_width = len(indices)

            lane_ok, lane_score = self.lane_confinement_check(gap_center_angle, gap_depth)
            if not lane_ok:
                continue

            world_yaw = wrap_to_pi(self.theta + gap_center_angle)

            if not self.footprint_is_map_safe(world_yaw, max_distance=min(gap_depth, 0.8), step=0.10):
                continue

            map_score = self.score_map_ray(world_yaw, max_distance=min(gap_depth, 2.0), step=0.15)

            score = (
                depth_weight * gap_depth
                + width_weight * gap_width
                - angle_weight * abs(gap_center_angle)
                + lane_score
                + map_score
            )

            if score > best_score:
                best_score = score
                best_gap = (gap_center_angle, indices, gap_depth)

        return best_gap
    
    # This function checks if the path ahead is clear or not. If it is, then I would like track_follow to take over
    def path_ahead_is_clear(self, forward_ranges, forward_angles, clear_distance=7.0):
        front_angle = math.radians(20.0)  # narrow forward cone

        front_mask = np.abs(forward_angles) < front_angle
        front_ranges = forward_ranges[front_mask]

        if front_ranges.size == 0:
            return False  # safer than True

        clear_ratio = np.mean(front_ranges > clear_distance)
        min_front = float(np.min(front_ranges))

        return clear_ratio > 0.80 and min_front > 0.6

    def scan_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02)
            )
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y

            q = transform.transform.rotation
            self.theta = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

        except TransformException as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            self.stop_robot()
            return

        ranges = np.array(msg.ranges, dtype=np.float32)

        if ranges.size == 0:
            self.get_logger().warn('Received empty LaserScan')
            self.stop_robot()
            return

        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max
        ranges = np.clip(ranges, msg.range_min, msg.range_max)

        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment

        # Forward 180 degrees
        forward_mask = (angles >= -math.pi / 2.0) & (angles <= math.pi / 2.0)
        forward_ranges = ranges[forward_mask]
        forward_angles = angles[forward_mask]

        if self.path_ahead_is_clear(forward_ranges, forward_angles):
            self.get_logger().info('Path ahead is clear, yielding to track follower')
            # Send message to the behaviour tree node to indicate that the path ahead is clear and that the robot can yield to the track follower node. This can be used to trigger a switch back to the track following strategy if the path ahead is clear.
            follow_the_gap_msg_recommendation = NavigationRecommendation()
            follow_the_gap_msg_recommendation.source = NavigationRecommendation.FOLLOW_THE_GAP
            follow_the_gap_msg_recommendation.valid = False  # Path ahead is clear, no need for FTG recommendation
            follow_the_gap_msg_recommendation.confidence = 0.0  # No confidence in needing FTG recommendation
            follow_the_gap_msg_recommendation.linear_x = 0.0
            follow_the_gap_msg_recommendation.angular_z = 0.0
            follow_the_gap_msg_recommendation.reason = NavigationRecommendation.NO_OBSTACLE  # Custom field to indicate reason for yielding, can be used by the behaviour tree node to make informed decisions

            self.behaviour_tree_publisher.publish(follow_the_gap_msg_recommendation)
            return

        left_sector = forward_ranges[forward_angles > 0.25]
        right_sector = forward_ranges[forward_angles < -0.25]

        left_min = float(np.min(left_sector)) if left_sector.size > 0 else msg.range_max
        right_min = float(np.min(right_sector)) if right_sector.size > 0 else msg.range_max

        side_bias = 0.0
        if right_min < 1.0:
            side_bias += 0.6 * (1.0 - right_min)
        if left_min < 1.0:
            side_bias -= 0.6 * (1.0 - left_min)

        if forward_ranges.size == 0:
            self.get_logger().warn('No forward-facing LiDAR points available')
            self.stop_robot()
            return

        obstacle_risk = self.compute_obstacle_risk(forward_ranges)
        self.ftg_confidence_weight = np.clip(
            self.min_ftg_confidence + (self.max_ftg_confidence - self.min_ftg_confidence) * obstacle_risk,
            0.0,
            1.0
        )

        disparities = self.find_disparities(forward_ranges)
        extended_ranges = self.extend_disparity(forward_ranges, disparities, msg.angle_increment)

        safe_mask = extended_ranges >= self.safe_distance

        safe_groups = [
            (key, [idx for idx, _ in group])
            for key, group in groupby(enumerate(safe_mask), key=lambda x: x[1])
        ]

        best_gap = self.score_gap(extended_ranges, safe_groups, forward_angles, obstacle_risk=obstacle_risk)

        self.get_logger().info(f'<DEBUG> Best gap: {best_gap}')

        if best_gap is None:
            self.get_logger().warn('No safe gap found')
            self.stop_robot()

            # Send message to the behaviour tree node to indicate that the path is blocked and that the robot is stopping to re-evaluate the environment. This can be used to trigger a switch to a different navigation strategy or to trigger a recovery behavior if the robot is stuck.
            follow_the_gap_msg_recommendation = NavigationRecommendation()
            follow_the_gap_msg_recommendation.source = NavigationRecommendation.FOLLOW_THE_GAP
            follow_the_gap_msg_recommendation.valid = False  # Path is blocked, not safe to proceed
            follow_the_gap_msg_recommendation.confidence = 0.0  # No confidence in proceeding forward
            follow_the_gap_msg_recommendation.linear_x = 0.0
            follow_the_gap_msg_recommendation.angular_z = 0.0
            follow_the_gap_msg_recommendation.reason = NavigationRecommendation.OBSTACLE_DETECTED  # Custom field to indicate reason for stopping, can be used by the behaviour tree node to make informed decisions

            self.behaviour_tree_publisher.publish(follow_the_gap_msg_recommendation)

            self.current_state = States.NO_SAFE_GAPS  # Update state to indicate no safe gaps found
            return

        gap_center_angle, gap_indices, gap_depth = best_gap

        gap_mid_idx = gap_indices[len(gap_indices) // 2]
        gap_distance = float(extended_ranges[gap_mid_idx])

        # Steering directly toward the chosen gap
        angular_z = self.steering_gain * gap_center_angle + side_bias
        angular_z = max(-self.max_turn_rate, min(self.max_turn_rate, angular_z))

        # Base speed from clearance
        linear_x = min(self.max_speed, 0.18 * gap_distance)

        # Slow down for harder turns
        turn_scale = max(0.25, 1.0 - min(abs(gap_center_angle) / 1.2, 0.75))
        linear_x *= turn_scale

        if abs(gap_center_angle) > 0.35:
            linear_x = min(linear_x, 0.20)

        if abs(gap_center_angle) > 0.60:
            linear_x = min(linear_x, 0.12)

        # If obstacle is very close ahead, slow more
        centre_idx = len(forward_ranges) // 2
        ahead_window = forward_ranges[max(0, centre_idx - 10): min(len(forward_ranges), centre_idx + 11)]
        min_ahead = float(np.min(ahead_window)) if ahead_window.size > 0 else msg.range_max

        if min_ahead < 1.0:
            linear_x = min(linear_x, 0.25)
        if min_ahead < 0.6:
            linear_x = 0.0

        if gap_distance >= self.safe_distance:
            linear_x = max(self.min_speed, linear_x)

        # If the previous state was NO_SAFE_GAPS and we have now found a safe gap and are in GAP_FOLLOWING and there are no obstacles very close ahead, then tell the behaviour tree node to use the track follower node again as the path ahead is now clear and safe to follow, otherwise if there are obstacles close ahead, then we can still follow the gap but we shouldn't switch back to track follower yet as the path isn't fully clear
        if self.previous_state == States.NO_SAFE_GAPS and self.current_state == States.GAP_FOLLOWING and self.path_ahead_is_clear(forward_ranges, forward_angles):
            self.get_logger().info('Transition: NO_SAFE_GAPS -> GAP_FOLLOWING - Found a safe gap, resuming movement')
            

        # If self.recommendation_config is True, publish the recommended cmd_vel to the behaviour tree node, otherwise just execute the cmd_vel without publishing to the behaviour tree node. This allows for testing the FTG algorithm in isolation without affecting the overall behaviour tree logic.
        if self.recommendation_config:
            # self.get_logger().info('Publishing Follow the Gap recommendation to behaviour tree node\n===================================================================================================================================================================')
            follow_the_gap_msg_recommendation = NavigationRecommendation()
            follow_the_gap_msg_recommendation.source = NavigationRecommendation.FOLLOW_THE_GAP
            follow_the_gap_msg_recommendation.valid = True
            follow_the_gap_msg_recommendation.confidence = float(self.ftg_confidence_weight)
            follow_the_gap_msg_recommendation.linear_x = linear_x
            follow_the_gap_msg_recommendation.angular_z = angular_z
            self.behaviour_tree_publisher.publish(follow_the_gap_msg_recommendation)

            # Saving the current state
            self.current_state = States.GAP_FOLLOWING
        else:
            self.publish_cmd(linear_x, angular_z)


        # Saving the previous state as the current state
        self.previous_state = self.current_state
    
def main():
    rclpy.init()
    node = FollowTheGap()
    rclpy.spin(node)
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()