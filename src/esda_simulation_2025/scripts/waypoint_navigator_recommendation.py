#!/usr/bin/env python3

from email.header import Header
from std_msgs.msg import Header as ROSHeader
import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs import msg
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan, PointCloud2, PointField  
import sensor_msgs_py.point_cloud2 as pc2
from tf2_geometry_msgs import do_transform_point
from nav_msgs.msg import Odometry



class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        # Declaring parameters for the waypoint navigator
        self.declare_parameter('map_topic', '/map') # Subscribes to the map topic to get the occupancy grid
        self.declare_parameter('robot_frame', 'base_link') # Subscribes to the robot frame to get the robot's current pose
        self.declare_parameter('frame_id', 'map') # Subscribes to the map frame to get the map's current pose
        self.declare_parameter('odometry_topic', '/odom') # Subscribes to the odometry topic to get the robot's current velocity
        self.declare_parameter('lane_topic', '/lane_markers') # Subscribes to the lane topic to get the lane markers
        
        self.declare_parameter('safety_bubble_radius', 0.5) # Safety bubble radius around the robot to avoid collisions

        # self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Getting the parameter values
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self.safety_bubble_radius = self.get_parameter('safety_bubble_radius').get_parameter_value().double_value

        # Transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Data relating to the robot's current pose and the map data (SLAM)
        self.current_pose = None
        self.map_data = None

        # Data relating to the robot's current velocity
        self.current_velocity = None

        # Subscribing to the map topic to get the occupancy grid
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        # Subscribing to the odometry topic to get the robot's current velocity
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.odometry_callback,
            10
        )

        self.lane_subscriber = self.create_subscription(
            MarkerArray,
            self.get_parameter('lane_topic').get_parameter_value().string_value,
            self.lane_callback,
            10
        )

        # Basic Navigator object to handle navigation tasks
        self.navigator = BasicNavigator()

        # Initial parameters / pose
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = self.frame_id
        self.initial_pose.header.stamp = self.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.position.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.navigator.setInitialPose(self.initial_pose)

        # Initial feedback and result for the navigation task
        self.feedback = None # Feedback from the navigation task. Set to None
        self.result = None # Result of the navigation task. Set to None

        # Goal pose for the robot to navigate to
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = self.frame_id
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 1.0
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.w = 1.0

        # Most recently calculated forward waypoint
        self.latest_forward_goal = None

        # Last waypoint actually sent to Nav2
        self.last_sent_goal = None

        # Prevent multiple goals being sent simultaneously
        self.goal_in_progress = False

        # Recalculate from map callbacks, but only send at a controlled rate
        self.goal_timer = self.create_timer(
            2.0,
            self.send_latest_forward_goal
        )

        self.initial_goal_timer = self.create_timer(
            5.0,
            self.send_initial_forward_goal
        )

        self.initial_forward_goal_sent = False

        self.raw_grid = [] # Raw occupancy grid data as a 2D numpy array
        self.map_matrix = [] # Occupancy grid data as a 2D numpy array

        # Parameters for lane following
        # Latest lane centreline expressed in map frame
        self.lane_centreline = []

        # Prevent old lane detections being used indefinitely
        self.last_lane_update_time = None

        # Parameters for the laser scan
        self.latest_scan = None

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.planning_timer = self.create_timer(
            0.5,
            self.update_forward_goal
        )

        # Final goal pose for the robot to navigate to (not used in this code, but can be set externally)
        self.final_goal = PoseStamped()
        self.final_goal.header.frame_id = self.frame_id
        self.final_goal.header.stamp = self.get_clock().now().to_msg()
        self.final_goal.pose.position.x = 4.168
        self.final_goal.pose.position.y = 29.937
        self.final_goal.pose.position.z = 0.0
        self.final_goal.pose.orientation.w = 1.0

        # Recovery parameters
        self.enter_recovery_mode = False

        # Time at which we first failed to generate normal waypoint 
        self.no_candidate_since = None

        # How long to tolerate continuous failure to generate a normal waypoint before entering recovery mode
        self.no_candidate_timeout = 20.0 # Seconds

        # Prevent repeatedly firing far goal attempts
        self.far_goal_in_progress = False

        # Subscribe to the global costmap
        self.global_costmap_subscriber = self.create_subscription(
            Costmap,
            '/global_costmap/costmap',
            self.global_costmap_callback,
            10
        )

        self.navigation_timer = self.create_timer(
            0.2,
            self.check_navigation_complete
        )

        self.gps_waypoints = []

        # Travel history to prevent sending waypoints behind robot
        self.travel_history_publisher = self.create_publisher(
            PointCloud2,
            '/travel_history',
            10
        )

        self.travel_history_points = []
        self.last_travel_marker = None
        self.travel_marker_spacing = 2.5

    def send_goal(self, goal_pose: PoseStamped, mode="normal"):
        """
        Send a Nav2 goal using BasicNavigator.

        mode:
            "normal"   -> normal lane/forward waypoint
            "recovery" -> recovery waypoint
        """

        if self.goal_in_progress:
            self.get_logger().warn(
                "Cannot send goal: another Nav2 goal is already active."
            )
            return

        goal_pose.header.frame_id = self.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().warn(
            f"Sending {mode} goal: "
            f"x={goal_pose.pose.position.x:.2f}, "
            f"y={goal_pose.pose.position.y:.2f}"
        )

        self.goal_in_progress = True
        self.navigation_mode = mode
        self.last_sent_goal = goal_pose

        self.navigator.goToPose(goal_pose)

    def send_initial_forward_goal(self):
        if self.initial_forward_goal_sent:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,       # target: map
                self.robot_frame,    # source: base_link
                rclpy.time.Time()
            )

        except TransformException as ex:
            self.get_logger().info(
                f"Waiting for map -> base_link TF: {ex}"
            )
            return

        # Current robot pose in map frame
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        robot_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        initial_distance = 1.0  # metres forward

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = (
            robot_x + initial_distance * math.cos(robot_yaw)
        )

        goal.pose.position.y = (
            robot_y + initial_distance * math.sin(robot_yaw)
        )

        goal.pose.position.z = 0.0

        # Keep the robot facing forward
        goal.pose.orientation.z = math.sin(robot_yaw / 2.0)
        goal.pose.orientation.w = math.cos(robot_yaw / 2.0)

        self.get_logger().info(
            f"Sending initial forward goal: "
            f"x={goal.pose.position.x:.2f}, "
            f"y={goal.pose.position.y:.2f}"
        )

        self.initial_forward_goal_sent = True
        self.initial_goal_timer.cancel()

        self.send_goal(
            goal,
            mode="normal"
        )

    def pull_towards_gps_waypoints(self):
        pass

    def send_latest_forward_goal(self):
        """
        Send the most recently calculated forward waypoint.

        A new goal is only sent if:
        - a valid forward waypoint exists;
        - no previous goal is currently being submitted;
        - the waypoint has moved far enough from the last sent goal.
        """
        if self.enter_recovery_mode:
            self.get_logger().warn(
                "Waypoint sending locked: "
                "robot is in recovery mode."
            )
            return

        if not self.initial_forward_goal_sent:
            self.get_logger().debug("Initial forward goal not sent yet.")
            return

        if self.latest_forward_goal is None:
            self.get_logger().debug("Nothing sent to Nav2: latest_forward_goal is None.")
            return

        goal_x = self.latest_forward_goal.pose.position.x
        goal_y = self.latest_forward_goal.pose.position.y

        # Do not keep resending almost exactly the same waypoint
        if self.last_sent_goal is not None:
            last_x = self.last_sent_goal.pose.position.x
            last_y = self.last_sent_goal.pose.position.y

            target_change = math.hypot(
                goal_x - last_x,
                goal_y - last_y
            )

            minimum_goal_change = 0.1

            if target_change < minimum_goal_change:
                return

        if self.goal_in_progress:
            return

        self.get_logger().info(
            f"Sending forward waypoint: x={goal_x:.2f}, y={goal_y:.2f}"
        )

        self.send_goal(
            self.latest_forward_goal,
            mode="normal"
        )

    def odometry_callback(self, msg: Odometry):
        self.current_velocity = msg.twist.twist.linear

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Convert occupancy-grid data into a 2D array.
        self.raw_grid = np.array(msg.data, dtype=np.int8)
        self.map_matrix = self.raw_grid.reshape((height, width))

    def update_forward_goal(self):
        if self.enter_recovery_mode:
            self.get_logger().warn(
                "Waypoint generation locked: "
                "robot is in recovery mode."
            )

            return

        # If the robot is still navigating to a previous waypoint, do not generate a new one.
        if self.goal_in_progress:
            if self.last_sent_goal is not None:
                goal_x = self.last_sent_goal.pose.position.x
                goal_y = self.last_sent_goal.pose.position.y

                distance_to_goal = math.hypot(
                    goal_x - self.robot_x,
                    goal_y - self.robot_y
                )

                self.get_logger().warn(
                    f"Waypoint generation locked: "
                    f"goal still marked active. "
                    f"Approximate distance={distance_to_goal:.2f} m"
                )
            else:
                self.get_logger().warn(
                    "Waypoint generation locked because "
                    "goal_in_progress=True, but no last goal exists."
                )

            return

        self.latest_forward_goal = None

        if self.map_data is None:
            self.get_logger().warn(
                "No occupancy grid data available yet."
            )
            return
        
        msg = self.map_data

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        try:
            # Get the robot pose in the map frame.
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,       # map
                self.robot_frame,    # base_link
                rclpy.time.Time()
            )

        except TransformException as ex:
            self.get_logger().warn(
                f"TF not ready yet: {ex}",
                throttle_duration_sec=2.0
            )
            return

        # Robot position in the map frame.
        self.robot_x = transform.transform.translation.x
        self.robot_y = transform.transform.translation.y


    

        # Robot orientation in the map frame.
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        self.robot_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        self.current_pose = (self.robot_x, self.robot_y, self.robot_yaw)

        self.update_travel_history()

        lane_goal = self.calculate_lane_goal()


        if lane_goal is not None:
            self.latest_forward_goal = lane_goal
            return

        right_clearance = self.get_scan_clearance(-100, -20)
        front_clearance = self.get_scan_clearance(-20, 20)
        left_clearance = self.get_scan_clearance(20, 100)

        if right_clearance < 1.5:
            self.get_logger().warn(
                f"Lane goal unavailable and right wall is only "
                f"{right_clearance:.2f} m away. Refusing forward fallback."
            )

            recovery_goal = self.find_left_recovery_goal()

            if recovery_goal is not None:
                self.latest_forward_goal = recovery_goal
                return
            
            self.get_logger().warn(
                "No safe left recovery waypoint found."
            )
            return

        # Convert robot position into occupancy-grid coordinates.
        robot_grid_x = int(
            (self.robot_x - origin_x) / resolution
        )
        robot_grid_y = int(
            (self.robot_y - origin_y) / resolution
        )

        # Make sure the robot itself is inside the map.
        if not (
            0 <= robot_grid_x < width
            and 0 <= robot_grid_y < height
        ):
            self.get_logger().warn(
                "Robot position is outside the occupancy grid."
            )
            return

        # Direction directly in front of the robot.
        forward_vx = math.cos(self.robot_yaw)
        forward_vy = math.sin(self.robot_yaw)

        # Search farther ahead, but only send a nearby goal.
        search_distance = 2.5
        max_goal_distance = 1.5
        minimum_goal_distance = 0.5

        # Permit only a small amount of unknown space.
        
        # This allows the robot to continue exploring without selecting a goal
        # several metres into completely unobserved space.
        max_unknown_distance = 0.8
        unknown_distance = 0.0

        step_size = resolution
        number_of_steps = int(search_distance / step_size)

        best_grid_x = robot_grid_x
        best_grid_y = robot_grid_y

        found_target = False

        for i in range(1, number_of_steps + 1):
            current_distance = i * step_size

            check_world_x = (
                self.robot_x
                + forward_vx * current_distance
            )
            check_world_y = (
                self.robot_y
                + forward_vy * current_distance
            )

            check_grid_x = int(
                (check_world_x - origin_x) / resolution
            )
            check_grid_y = int(
                (check_world_y - origin_y) / resolution
            )

            # Stop if the ray leaves the occupancy grid.
            if not (
                0 <= check_grid_x < width
                and 0 <= check_grid_y < height
            ):
                break

            cell_value = self.map_matrix[
                check_grid_y,
                check_grid_x
            ]

            if cell_value == 0:
                # Confirmed free space.
                best_grid_x = check_grid_x
                best_grid_y = check_grid_y
                found_target = True

                # Reset because we have returned to known free space.
                unknown_distance = 0.0

            elif cell_value == -1:
                # Unknown space.
                unknown_distance += step_size

                if unknown_distance <= max_unknown_distance:
                    best_grid_x = check_grid_x
                    best_grid_y = check_grid_y
                    found_target = True
                else:
                    break

            else:
                # Positive values represent occupied/probably occupied cells.
                break

        if not found_target:
            self.get_logger().warn(
                "No suitable forward target found."
            )
            return

        # Convert selected grid cell back into map coordinates.
        best_world_x = (
            origin_x
            + (best_grid_x + 0.5) * resolution
        )
        best_world_y = (
            origin_y
            + (best_grid_y + 0.5) * resolution
        )

        dx = best_world_x - self.robot_x
        dy = best_world_y - self.robot_y

        distance_to_target = math.hypot(dx, dy)

        # Limit the actual Nav2 goal to a short look-ahead distance.
        if distance_to_target > max_goal_distance:
            scale = max_goal_distance / distance_to_target

            best_world_x = self.robot_x + dx * scale
            best_world_y = self.robot_y + dy * scale

            distance_to_target = max_goal_distance

        if distance_to_target < minimum_goal_distance:
            self.get_logger().info(
                f"Forward target is too close: "
                f"{distance_to_target:.2f} m"
            )

            # Do not clear latest_forward_goal here.
            # The next map update may produce a valid target.
            return

        forward_goal = PoseStamped()

        forward_goal.header.frame_id = self.frame_id
        forward_goal.header.stamp = (
            self.get_clock().now().to_msg()
        )

        forward_goal.pose.position.x = best_world_x
        forward_goal.pose.position.y = best_world_y
        forward_goal.pose.position.z = 0.0


        # Face toward the waypoint
        goal_yaw = math.atan2(
            best_world_y - self.robot_y,
            best_world_x - self.robot_x
        )

        # Make the goal face in the current forward direction.
        forward_goal.pose.orientation.x = 0.0
        forward_goal.pose.orientation.y = 0.0
        forward_goal.pose.orientation.z = math.sin(
            goal_yaw / 2.0
        )
        forward_goal.pose.orientation.w = math.cos(
            goal_yaw / 2.0
        )

        if self.is_waypoint_within_safety_bubble(forward_goal):
            self.get_logger().info(
                "Forward target is within safety bubble."
            )
            return

        # This goal will be sent by send_latest_forward_goal().
        self.latest_forward_goal = forward_goal

        target_cell_value = self.map_matrix[
            best_grid_y,
            best_grid_x
        ]

        self.get_logger().info(
            f"Robot: "
            f"x={self.robot_x:.2f}, "
            f"y={self.robot_y:.2f}, "
            f"yaw={self.robot_yaw:.2f} | "
            f"Forward goal: "
            f"x={best_world_x:.2f}, "
            f"y={best_world_y:.2f}, "
            f"distance={distance_to_target:.2f} m | "
            f"cell={target_cell_value}"
        )
    
    def is_waypoint_within_safety_bubble(self, waypoint: PoseStamped) -> bool:
        """
        Check if a given waypoint is within the safety bubble radius of the robot.

        :param waypoint: The waypoint to check.
        :return: True if the waypoint is within the safety bubble, False otherwise.
        """
        if self.current_pose is None:
            return False

        dx = waypoint.pose.position.x - self.current_pose[0]
        dy = waypoint.pose.position.y - self.current_pose[1]
        distance = math.hypot(dx, dy)

        self.get_logger().warn("Checking waypoint distance: {:.2f} m, safety bubble radius: {:.2f} m".format( distance, self.get_parameter('safety_bubble_radius').get_parameter_value().double_value))
        return distance <= self.get_parameter('safety_bubble_radius').get_parameter_value().double_value

    def lane_callback(self, msg: MarkerArray):
        left_points = []
        right_points = []

        for marker in msg.markers:
            
            source_frame = marker.header.frame_id

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id,       # target: map
                    source_frame,        # source: marker frame
                    rclpy.time.Time()
                )
            except TransformException as ex:
                self.get_logger().warn(
                    f"TF not ready for lane markers: {ex}",
                    throttle_duration_sec=2.0
                )
                continue 

            transformed_points = self.transform_marker_points(
                marker.points,
                source_frame,
                transform
            )

            for point_x, point_y in transformed_points:
                if marker.ns == "left_lane":
                    left_points.append((point_x, point_y))

                elif marker.ns == "right_lane":
                    right_points.append((point_x, point_y))


        self.get_logger().info(
            f"Left points: {len(left_points)}, "
            f"Right points: {len(right_points)}"
        )

        if len(left_points) < 2 or len(right_points) < 2:
            self.get_logger().warn(
                "Not enough left/right lane points."
            )
            self.lane_centreline = []
            self.last_lane_update_time = None
            return

        # Sort each boundary from near to far.
        left_points.sort(
            key=lambda point: math.hypot(
                point[0] - self.robot_x,
                point[1] - self.robot_y
            )
        )

        right_points.sort(
            key=lambda point: math.hypot(
                point[0] - self.robot_x,
                point[1] - self.robot_y
            )
        )

        number_of_pairs = min(
            len(left_points),
            len(right_points)
        )

        centreline = []

        for i in range(number_of_pairs):
            centre_x = (
                left_points[i][0]
                + right_points[i][0]
            ) / 2.0

            centre_y = (
                left_points[i][1]
                + right_points[i][1]
            ) / 2.0

            centreline.append(
                (centre_x, centre_y)
            )

        self.lane_centreline = self.smooth_centerline(
            centreline,
            window_size=3
        )

        self.get_logger().warn(
            f"Updated lane centreline with "
            f"{len(self.lane_centreline)} points"
        )

    def transform_marker_points(
        self,
        points,
        source_frame,
        transform
    ):
        transformed_points = []

        for point in points:
            stamped_point = PointStamped()
            stamped_point.header.frame_id = source_frame
            stamped_point.header.stamp = self.get_clock().now().to_msg()

            stamped_point.point.x = point.x
            stamped_point.point.y = point.y
            stamped_point.point.z = point.z

            transformed = do_transform_point(
                stamped_point,
                transform
            )

            transformed_points.append(
                (
                    transformed.point.x,
                    transformed.point.y
                )
            )

        return transformed_points
    
    def smooth_centerline(self, points, window_size=3):
        if len(points) < window_size:
            return points

        smoothed = []

        for i in range(len(points)):
            start = max(0, i - window_size + 1)
            window = points[start:i + 1]

            average_x = sum(
                point[0] for point in window
            ) / len(window)

            average_y = sum(
                point[1] for point in window
            ) / len(window)

            smoothed.append((average_x, average_y))

        return smoothed

    def calculate_lane_goal(self):
        """
        Generate a short lane-following waypoint while maintaining clearance
        from nearby walls.

        Positive lateral offset = robot's left.
        Negative lateral offset = robot's right.
        """

        if len(self.lane_centreline) < 2:
            return None

        if self.current_pose is None:
            return None

        # Robot-relative unit vectors expressed in the map frame.
        forward_x = math.cos(self.robot_yaw)
        forward_y = math.sin(self.robot_yaw)

        left_x = -math.sin(self.robot_yaw)
        left_y = math.cos(self.robot_yaw)

        # Current laser clearances.
        right_clearance = self.get_scan_clearance(-100, -20)
        front_clearance = self.get_scan_clearance(-20, 20)
        left_clearance = self.get_scan_clearance(20, 100)

        self.get_logger().info(
            f"SCAN CLEARANCE | "
            f"left={left_clearance:.2f}, "
            f"front={front_clearance:.2f}, "
            f"right={right_clearance:.2f}"
        )

        # ----------------------------------------------------------
        # 1. Keep only centreline points that are in front of robot
        # ----------------------------------------------------------

        forward_lane_points = []

        for point_x, point_y in self.lane_centreline:
            dx = point_x - self.robot_x
            dy = point_y - self.robot_y

            longitudinal = (
                dx * forward_x
                + dy * forward_y
            )

            lateral = (
                dx * left_x
                + dy * left_y
            )

            if longitudinal > 0.15:
                forward_lane_points.append(
                    (
                        point_x,
                        point_y,
                        longitudinal,
                        lateral
                    )
                )

        if len(forward_lane_points) < 2:
            self.get_logger().warn(
                "Not enough lane-centre points ahead of robot."
            )
            return None

        # Sort using forward distance, not Euclidean distance.
        forward_lane_points.sort(
            key=lambda point: point[2]
        )

        # ----------------------------------------------------------
        # 2. Choose look-ahead distances
        # ----------------------------------------------------------

        if front_clearance < 0.7:
            lookahead_distances = [0.30, 0.40, 0.50]

        elif (
            front_clearance < 1.2
            or right_clearance < 0.8
            or left_clearance < 0.8
        ):
            lookahead_distances = [0.40, 0.60, 0.80]

        else:
            lookahead_distances = [1.60, 1.90, 2.20]

        # ----------------------------------------------------------
        # 3. Choose permitted lateral offsets
        # ----------------------------------------------------------

        # Wall is close on the right: do not allow right-side goals.
        if right_clearance < 0.7:
            lateral_offsets = [
                0.70,
                0.90,
                1.10,
                1.30
            ]

        elif right_clearance < 1.2:
            lateral_offsets = [
                0.45,
                0.65,
                0.85,
                1.05
            ]

        elif right_clearance < 1.7:
            lateral_offsets = [
                0.20,
                0.40,
                0.60,
                0.80
            ]

        # Wall is close on the left: move toward the right.
        elif left_clearance < 0.7:
            lateral_offsets = [
                -1.30,
                -1.10,
                -0.90,
                -0.70
            ]

        elif left_clearance < 1.2:
            lateral_offsets = [
                -1.05,
                -0.85,
                -0.65,
                -0.45
            ]

        elif left_clearance < 1.7:
            lateral_offsets = [
                -0.80,
                -0.60,
                -0.40,
                -0.20
            ]

        # Both sides are reasonably clear.
        else:
            lateral_offsets = [
                -0.40,
                -0.20,
                0.00,
                0.20,
                0.40
            ]

        best_candidate = None
        best_score = -float("inf")

        # ----------------------------------------------------------
        # 4. Generate and evaluate candidates
        # ----------------------------------------------------------

        for requested_lookahead in lookahead_distances:

            # Find the lane-centre point closest to this forward distance.
            centre_point = min(
                forward_lane_points,
                key=lambda point: abs(
                    point[2] - requested_lookahead
                )
            )

            centre_x = centre_point[0]
            centre_y = centre_point[1]
            centre_longitudinal = centre_point[2]

            for lateral_offset in lateral_offsets:

                candidate_x = (
                    centre_x
                    + lateral_offset * left_x
                )

                candidate_y = (
                    centre_y
                    + lateral_offset * left_y
                )

                dx = candidate_x - self.robot_x
                dy = candidate_y - self.robot_y

                candidate_forward = (
                    dx * forward_x
                    + dy * forward_y
                )

                candidate_lateral = (
                    dx * left_x
                    + dy * left_y
                )

                candidate_distance = math.hypot(dx, dy)

                # Do not select positions behind or almost beside the robot.
                if candidate_forward < 0.20:
                    continue

                if candidate_distance < 0.35:
                    continue

                # Clearance only at the final waypoint.
                goal_clearance = self.get_map_clearance(
                    candidate_x,
                    candidate_y
                )

                # Minimum clearance along the line from robot to waypoint.
                path_clearance = self.get_path_clearance(
                    self.robot_x,
                    self.robot_y,
                    candidate_x,
                    candidate_y
                )

                # Hard rejection rather than merely reducing the score.
                if goal_clearance < 1:
                    continue

                if path_clearance < 0.9:
                    continue

                # --------------------------------------------------
                # Candidate scoring
                # --------------------------------------------------

                clearance_score = (
                    8.0 * goal_clearance
                    + 12.0 * path_clearance
                )

                progress_score = 1.5 * candidate_forward

                # Normally remain fairly close to the lane centre.
                lane_offset_penalty = 0.8 * abs(lateral_offset)

                wall_bias = 0.0

                # Explicitly reward moving left when right wall is close.
                if right_clearance < 1.7:
                    wall_closeness = max(
                        0.0,
                        1.7 - right_clearance
                    )

                    wall_bias += (
                        8.0
                        * wall_closeness
                        * max(0.0, candidate_lateral)
                    )

                    # Strong rejection if candidate remains on robot's right.
                    if candidate_lateral < 0.0:
                        wall_bias -= 20.0

                # Explicitly reward moving right when left wall is close.
                if left_clearance < 1.7:
                    wall_closeness = max(
                        0.0,
                        1.7 - left_clearance
                    )

                    wall_bias += (
                        8.0
                        * wall_closeness
                        * max(0.0, -candidate_lateral)
                    )

                    if candidate_lateral > 0.0:
                        wall_bias -= 20.0

                score = (
                    clearance_score
                    + progress_score
                    + wall_bias
                    - lane_offset_penalty
                )

                self.get_logger().info(
                    f"CANDIDATE | "
                    f"forward={candidate_forward:.2f}, "
                    f"lateral={candidate_lateral:.2f}, "
                    f"goal_clearance={goal_clearance:.2f}, "
                    f"path_clearance={path_clearance:.2f}, "
                    f"score={score:.2f}"
                )

                if score > best_score:
                    best_score = score

                    best_candidate = (
                        candidate_x,
                        candidate_y,
                        candidate_forward,
                        candidate_lateral,
                        goal_clearance,
                        path_clearance
                    )

        if best_candidate is None:
            self.get_logger().warn(
                "No safe lane-following candidate found."
            )
            return None

        (
            goal_x,
            goal_y,
            chosen_forward,
            chosen_lateral,
            chosen_goal_clearance,
            chosen_path_clearance
        ) = best_candidate

        free_space_result = self.bias_waypoint_toward_free_space(
            goal_x,
            goal_y,
            forward_x,
            forward_y,
            left_x,
            left_y
        )

        if free_space_result is None:
            self.get_logger().warn(
                "Rejecting waypoint because it could not be moved "
                "to a sufficiently clear position."
            )
            return None

        goal_x, goal_y = free_space_result

        # ----------------------------------------------------------
        # 5. Final hard wall-side check
        # ----------------------------------------------------------

        # Ensure the selected point is definitely to the left when
        # the right wall is extremely close.
        if right_clearance < 0.8 and chosen_lateral < 0.70:
            required_shift = 0.70 - chosen_lateral

            goal_x += required_shift * left_x
            goal_y += required_shift * left_y

            chosen_lateral = 0.70

            self.get_logger().error(
                "RIGHT WALL VERY CLOSE: "
                "forcing waypoint at least 0.70 m left."
            )

        # Recheck after the final adjustment.
        final_goal_clearance = self.get_map_clearance(
            goal_x,
            goal_y
        )

        final_path_clearance = self.get_path_clearance(
            self.robot_x,
            self.robot_y,
            goal_x,
            goal_y
        )

        if final_goal_clearance < 1.0:
            self.get_logger().warn(
                f"Final waypoint clearance too small: "
                f"{final_goal_clearance:.2f} m"
            )
            return None

        if final_path_clearance < 0.80:
            self.get_logger().warn(
                f"Final waypoint path too close to obstacle: "
                f"{final_path_clearance:.2f} m"
            )
            return None

        # Face directly toward the selected waypoint.
        # goal_yaw = self.robot_yaw

        goal_yaw = math.atan2(
            goal_y - self.robot_y,
            goal_x - self.robot_x
        )

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.orientation.w = math.cos(goal_yaw / 2.0)

        # Your existing code incorrectly called this function without
        # supplying the required waypoint argument.
        if self.is_waypoint_within_safety_bubble(goal):
            self.get_logger().warn(
                "Selected lane waypoint is too close to robot."
            )
            return None

        self.get_logger().warn(
            f"SELECTED GOAL | "
            f"forward={chosen_forward:.2f}, "
            f"lateral={chosen_lateral:.2f}, "
            f"goal_clearance={final_goal_clearance:.2f}, "
            f"path_clearance={final_path_clearance:.2f}, "
            f"x={goal_x:.2f}, "
            f"y={goal_y:.2f}"
        )

        return goal

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def get_scan_clearance(self, angle_min_deg: float, angle_max_deg: float)->float:

        if self.latest_scan is None:
            self.get_logger().warn("No laser scan data available.")
            return float('inf')

        valid_ranges = []

        angle_min = math.radians(angle_min_deg)
        angle_max = math.radians(angle_max_deg)

        for index, range_value in enumerate(self.latest_scan.ranges):
            angle = self.latest_scan.angle_min + index * self.latest_scan.angle_increment

            if angle_min <= angle <= angle_max:
                if not math.isinf(range_value) and not math.isnan(range_value):
                    valid_ranges.append(range_value)

        if not valid_ranges:
            self.get_logger().warn("No valid scan data in the specified angle range.")
            return float('inf')

        return min(valid_ranges)

    
    def get_map_clearance(self, world_x: float, world_y: float) -> float:
        """
        Estimate the distance from a world point to the nearest occupied
        occupancy-grid cell.
        """

        if self.map_data is None or len(self.map_matrix) == 0:
            return 0.0

        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        width = self.map_data.info.width
        height = self.map_data.info.height

        centre_grid_x = int((world_x - origin_x) / resolution)
        centre_grid_y = int((world_y - origin_y) / resolution)

        if not (
            0 <= centre_grid_x < width
            and 0 <= centre_grid_y < height
        ):
            return 0.0

        search_radius = 2.0
        search_cells = int(search_radius / resolution)

        minimum_distance = search_radius

        for grid_y in range(
            max(0, centre_grid_y - search_cells),
            min(height, centre_grid_y + search_cells + 1)
        ):
            for grid_x in range(
                max(0, centre_grid_x - search_cells),
                min(width, centre_grid_x + search_cells + 1)
            ):
                cell_value = self.map_matrix[grid_y, grid_x]

                # Treat occupied cells as obstacles.
                if cell_value >= 50:
                    obstacle_world_x = (
                        origin_x + (grid_x + 0.5) * resolution
                    )
                    obstacle_world_y = (
                        origin_y + (grid_y + 0.5) * resolution
                    )

                    distance = math.hypot(
                        obstacle_world_x - world_x,
                        obstacle_world_y - world_y
                    )

                    minimum_distance = min(
                        minimum_distance,
                        distance
                    )

        return minimum_distance
    
    def get_path_clearance(
        self,
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float
    ) -> float:
        """
        Return the minimum map clearance along the straight line between
        the robot and a candidate waypoint.
        """

        path_length = math.hypot(
            end_x - start_x,
            end_y - start_y
        )

        if path_length < 1e-6:
            return 0.0

        if self.map_data is None:
            return 0.0

        resolution = self.map_data.info.resolution

        # Sample at least every half map cell.
        sample_spacing = max(
            0.03,
            resolution * 0.5
        )

        number_of_samples = max(
            2,
            int(path_length / sample_spacing)
        )

        minimum_clearance = float("inf")

        # Start slightly ahead of the robot so that the robot's own
        # occupied footprint does not immediately produce zero clearance.
        for i in range(1, number_of_samples + 1):
            ratio = i / number_of_samples

            sample_x = (
                start_x
                + ratio * (end_x - start_x)
            )

            sample_y = (
                start_y
                + ratio * (end_y - start_y)
            )

            clearance = self.get_map_clearance(
                sample_x,
                sample_y
            )

            minimum_clearance = min(
                minimum_clearance,
                clearance
            )

        if math.isinf(minimum_clearance):
            return 0.0

        return minimum_clearance
    
    def bias_waypoint_toward_free_space(
        self,
        original_x: float,
        original_y: float,
        forward_x: float,
        forward_y: float,
        left_x: float,
        left_y: float
    ):
        """
        Search around the proposed waypoint and return a nearby point with
        greater obstacle clearance.

        Positive lateral offset moves left.
        Negative lateral offset moves right.
        """

        minimum_required_clearance = 1.20

        original_clearance = self.get_map_clearance(
            original_x,
            original_y
        )

        self.get_logger().warn(
            f"Original waypoint clearance: "
            f"{original_clearance:.2f} m"
        )

        # The original waypoint is already sufficiently clear.
        if original_clearance >= minimum_required_clearance:
            return original_x, original_y

        best_x = None
        best_y = None
        best_score = -float("inf")
        best_clearance = 0.0
        best_offset = 0.0

        # Search both sides of the original waypoint.
        lateral_offsets = [
            -1.50,
            -1.25,
            -1.00,
            -0.75,
            -0.50,
            -0.25,
            0.00,
            0.25,
            0.50,
            0.75,
            1.00,
            1.25,
            1.50
        ]

        # Also permit a small amount of forward/backward adjustment.
        longitudinal_offsets = [
            -0.20,
            0.00,
            0.20,
            0.40
        ]

        for longitudinal_offset in longitudinal_offsets:
            for lateral_offset in lateral_offsets:

                candidate_x = (
                    original_x
                    + longitudinal_offset * forward_x
                    + lateral_offset * left_x
                )

                candidate_y = (
                    original_y
                    + longitudinal_offset * forward_y
                    + lateral_offset * left_y
                )

                goal_clearance = self.get_map_clearance(
                    candidate_x,
                    candidate_y
                )

                path_clearance = self.get_path_clearance(
                    self.robot_x,
                    self.robot_y,
                    candidate_x,
                    candidate_y
                )

                # Reject unsafe candidates.
                if goal_clearance <0.6:
                    continue

                if path_clearance < 0.60:
                    continue

                dx = candidate_x - self.robot_x
                dy = candidate_y - self.robot_y

                candidate_forward = (
                    dx * forward_x
                    + dy * forward_y
                )

                # Never choose something behind the robot.
                if candidate_forward < 0.25:
                    continue

                # Clearance dominates the score.
                # Small offset penalty prevents unnecessary large jumps.
                score = (
                    20.0 * goal_clearance
                    + 15.0 * path_clearance
                    + 1.0 * candidate_forward
                    - 0.5 * abs(lateral_offset)
                    - 0.3 * abs(longitudinal_offset)
                )

                self.get_logger().info(
                    f"FREE-SPACE SEARCH | "
                    f"lateral={lateral_offset:.2f}, "
                    f"forward_adjustment={longitudinal_offset:.2f}, "
                    f"goal_clearance={goal_clearance:.2f}, "
                    f"path_clearance={path_clearance:.2f}, "
                    f"score={score:.2f}"
                )

                if score > best_score:
                    best_score = score
                    best_x = candidate_x
                    best_y = candidate_y
                    best_clearance = goal_clearance
                    best_offset = lateral_offset

        if best_x is None:
            self.get_logger().error(
                "Waypoint is near a wall and no safer nearby "
                "free-space position was found."
            )

            return None

        self.get_logger().warn(
            f"WAYPOINT MOVED TOWARD FREE SPACE | "
            f"lateral shift={best_offset:.2f} m, "
            f"old clearance={original_clearance:.2f} m, "
            f"new clearance={best_clearance:.2f} m"
        )

        return best_x, best_y
    
    def find_left_recovery_goal(self):
        if self.map_data is None:
            return None
        
        forward_x = math.cos(self.robot_yaw)
        forward_y = math.sin(self.robot_yaw)

        left_x = -math.sin(self.robot_yaw)
        left_y = math.cos(self.robot_yaw)

        best_goal = None
        best_score = -float("inf")

        # Not searching directly adjacent to the robot because that area is likely to be occupied.
        forward_distances = [
            0.4,
            0.6,
            0.8,
            1.0,
            1.2
        ]

        # Positive = robot's left, Negative = robot's right
        left_offsets = [
            0.3,
            0.5,
            0.7,
            0.9,
            1.1
        ]

        for forward_distance in forward_distances:
            for left_offset in left_offsets:

                candidate_x = (
                    self.robot_x
                    + forward_distance * forward_x
                    + left_offset * left_x
                )

                candidate_y = (
                    self.robot_y
                    + forward_distance * forward_y
                    + left_offset * left_y
                )

                # Clearance at candidate
                goal_clearance = self.get_map_clearance(
                    candidate_x,
                    candidate_y
                )

                # Clearance along path to candidate
                path_clearance = self.get_path_clearance(
                    self.robot_x,
                    self.robot_y,
                    candidate_x,
                    candidate_y
                )

                self.get_logger().warn(
                    f"RECOVERY CANDIDATE | "
                    f"forward={forward_distance:.2f}, "
                    f"left={left_offset:.2f}, "
                    f"goal_clearance={goal_clearance:.2f}, "
                    f"path_clearance={path_clearance:.2f}"
                )



                # Hard safety rejection
                if goal_clearance < 0.7:
                    continue

                if path_clearance < 0.2:
                    continue

                # Prefer:
                # 1. high obstacle clearance
                # 2. moving left
                # 3. some forward progress
                score = (
                    10.0 * goal_clearance
                    + 8.0 * path_clearance
                    + 3.0 * left_offset
                    + 1.0 * forward_distance
                )

                if score > best_score:
                    best_score = score
                    best_goal = (
                        candidate_x,
                        candidate_y
                    )

        if best_goal is None:
            return None

        goal_x, goal_y = best_goal

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0

        # Face toward recovery waypoint
        goal_yaw = math.atan2(
            goal_y - self.robot_y,
            goal_x - self.robot_x
        )

        goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.orientation.w = math.cos(goal_yaw / 2.0)

        self.get_logger().warn(
            f"LEFT RECOVERY GOAL | "
            f"x={goal_x:.2f}, "
            f"y={goal_y:.2f}"
        )

        return goal

    def robot_recovery(self):
        """
        Search the front half of the robot for a recovery goal that:
        1. is locally safe,
        2. is in front of the robot,
        3. can actually be planned to by Nav2.
        """

        self.get_logger().warn(
            "Recovery mode: searching for a reachable front-space goal."
        )

        recovery_goal = self.find_front_recovery_goal()

        if recovery_goal is None:
            self.get_logger().error(
                "No reachable recovery goal found."
            )
            self.enter_recovery_mode = True
            self.schedule_recovery_retry()
            return

        self.get_logger().warn(
            f"Sending recovery goal: "
            f"x={recovery_goal.pose.position.x:.2f}, "
            f"y={recovery_goal.pose.position.y:.2f}"
        )

        self.send_goal(
            recovery_goal,
            mode="recovery"
        )

    def retry_recovery_goal(self):
        self.recovery_retry_timer.cancel()
        del self.recovery_retry_timer

        if not self.enter_recovery_mode:
            return

        if self.goal_in_progress:
            return

        self.robot_recovery()

    def find_front_recovery_goal(self):
        if self.map_data is None:
            return None

        # ----------------------------------------------------------
        # Get the CURRENT robot pose
        # ----------------------------------------------------------

        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_frame,
                rclpy.time.Time()
            )

        except TransformException as ex:
            self.get_logger().warn(
                f"Could not get current robot pose for recovery: {ex}"
            )
            return None

        self.robot_x = transform.transform.translation.x
        self.robot_y = transform.transform.translation.y

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        self.robot_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        self.current_pose = (
            self.robot_x,
            self.robot_y,
            self.robot_yaw
        )

        # ----------------------------------------------------------
        # Current start pose for Nav2 planner
        # ----------------------------------------------------------

        start_pose = PoseStamped()

        start_pose.header.frame_id = self.frame_id
        start_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        start_pose.pose.position.x = self.robot_x
        start_pose.pose.position.y = self.robot_y
        start_pose.pose.position.z = 0.0

        start_pose.pose.orientation.x = 0.0
        start_pose.pose.orientation.y = 0.0
        start_pose.pose.orientation.z = math.sin(
            self.robot_yaw / 2.0
        )
        start_pose.pose.orientation.w = math.cos(
            self.robot_yaw / 2.0
        )

        # ----------------------------------------------------------
        # Map information
        # ----------------------------------------------------------

        msg = self.map_data

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # ----------------------------------------------------------
        # Generate candidate goals
        # ----------------------------------------------------------

        candidates = []

        search_distances = [
            0.8,
            1.0,
            1.2,
            1.5
        ]

        search_angles_deg = [
            -75,
            -60,
            -45,
            -30,
            -15,
            0,
            15,
            30,
            45,
            60,
            75
        ]

        forward_x = math.cos(self.robot_yaw)
        forward_y = math.sin(self.robot_yaw)

        for distance in search_distances:
            for angle_deg in search_angles_deg:

                angle_offset = math.radians(angle_deg)

                candidate_yaw = (
                    self.robot_yaw
                    + angle_offset
                )

                candidate_x = (
                    self.robot_x
                    + distance * math.cos(candidate_yaw)
                )

                candidate_y = (
                    self.robot_y
                    + distance * math.sin(candidate_yaw)
                )

                # --------------------------------------------------
                # Grid check
                # --------------------------------------------------

                grid_x = int(
                    (candidate_x - origin_x) / resolution
                )

                grid_y = int(
                    (candidate_y - origin_y) / resolution
                )

                if not (
                    0 <= grid_x < width
                    and 0 <= grid_y < height
                ):
                    continue

                cell_value = self.map_matrix[
                    grid_y,
                    grid_x
                ]

                if cell_value >= 50:
                    continue

                # --------------------------------------------------
                # Make sure it is ACTUALLY in front
                # --------------------------------------------------

                dx = candidate_x - self.robot_x
                dy = candidate_y - self.robot_y

                forward_progress = (
                    dx * forward_x
                    + dy * forward_y
                )

                if forward_progress <= 0.20:
                    continue

                # --------------------------------------------------
                # Your own safety checks
                # --------------------------------------------------

                goal_clearance = self.get_map_clearance(
                    candidate_x,
                    candidate_y
                )

                path_clearance = self.get_path_clearance(
                    self.robot_x,
                    self.robot_y,
                    candidate_x,
                    candidate_y
                )

                if goal_clearance < 0.8:
                    continue

                if path_clearance < 0.6:
                    continue

                # --------------------------------------------------
                # Candidate scoring
                # --------------------------------------------------

                clearance_score = (
                    4.0 * goal_clearance
                    + 5.0 * path_clearance
                )

                forward_score = (
                    8.0 * forward_progress
                )

                angle_penalty = (
                    0.05 * abs(angle_deg)
                )

                unknown_penalty = 0.0

                if cell_value == -1:
                    unknown_penalty = 2.0

                score = (
                    clearance_score
                    + forward_score
                    - angle_penalty
                    - unknown_penalty
                )

                goal = PoseStamped()

                goal.header.frame_id = self.frame_id
                goal.header.stamp = (
                    self.get_clock().now().to_msg()
                )

                goal.pose.position.x = candidate_x
                goal.pose.position.y = candidate_y
                goal.pose.position.z = 0.0

                goal.pose.orientation.x = 0.0
                goal.pose.orientation.y = 0.0
                goal.pose.orientation.z = math.sin(
                    candidate_yaw / 2.0
                )
                goal.pose.orientation.w = math.cos(
                    candidate_yaw / 2.0
                )

                candidates.append(
                    (
                        score,
                        goal,
                        angle_deg,
                        distance,
                        goal_clearance,
                        path_clearance,
                        forward_progress
                    )
                )

        if not candidates:
            self.get_logger().warn(
                "No locally-safe recovery candidates found."
            )
            return None

        # Best-scoring candidates first.
        candidates.sort(
            key=lambda item: item[0],
            reverse=True
        )

        # ----------------------------------------------------------
        # Ask NAV2 if each candidate is actually reachable
        # ----------------------------------------------------------

        for (
            score,
            goal,
            angle_deg,
            distance,
            goal_clearance,
            path_clearance,
            forward_progress
        ) in candidates:

            self.get_logger().warn(
                f"CHECKING NAV2 PATH | "
                f"angle={angle_deg:+.0f} deg, "
                f"distance={distance:.2f}, "
                f"score={score:.2f}"
            )

            try:
                path = self.navigator.getPath(
                    start_pose,
                    goal
                )

            except Exception as ex:
                self.get_logger().warn(
                    f"Nav2 path check failed: {ex}"
                )
                continue

            # No global path exists.
            if path is None:
                self.get_logger().warn(
                    f"Rejected recovery candidate: "
                    f"Nav2 cannot plan to it."
                )
                continue

            # Optionally also reject an empty path.
            if len(path.poses) == 0:
                self.get_logger().warn(
                    "Rejected recovery candidate: "
                    "Nav2 returned an empty path."
                )
                continue

            self.get_logger().warn(
                f"VALID RECOVERY GOAL FOUND | "
                f"angle={angle_deg:+.0f} deg, "
                f"distance={distance:.2f} m, "
                f"forward={forward_progress:.2f} m, "
                f"goal_clearance={goal_clearance:.2f}, "
                f"path_clearance={path_clearance:.2f}, "
                f"path_points={len(path.poses)}"
            )

            return goal

        self.get_logger().error(
            "All recovery candidates were rejected by Nav2 planner."
        )

        return None

    def update_travel_history(self):
        if self.current_pose is None:
            return
        
        current_x = self.robot_x
        current_y = self.robot_y

        # First last travel marker append
        if self.last_travel_marker is None:
            self.last_travel_marker = (current_x, current_y)

            self.travel_history_points.append(
                (current_x, current_y)
            )

            self.publish_travel_history_marker()
            return
        
        last_x, last_y = self.last_travel_marker

        distance_moved = math.hypot(
            current_x - last_x,
            current_y - last_y
        )

        if distance_moved < self.travel_marker_spacing:
            return
        
        self.last_travel_marker = (current_x, current_y)
        self.travel_history_points.append((current_x, current_y))

        self.publish_travel_history_marker()

    def publish_travel_history_marker(self):
        if not self.travel_history_points:
            return

        header = ROSHeader()
        header.frame_id = self.frame_id
        header.stamp = (self.get_clock().now().to_msg())

        cloud = pc2.create_cloud_xyz32(
            header,
            [(x, y, 0.0) for x, y in self.travel_history_points]
        )

        self.travel_history_publisher.publish(
            cloud
        )

    def check_navigation_complete(self):
        if not self.goal_in_progress:
            return

        # Get feedback while the task is active
        feedback = self.navigator.getFeedback()

        if feedback is not None:
            self.get_logger().info(
                f"NAV2 FEEDBACK | "
                f"mode={self.navigation_mode}, "
                f"distance_remaining="
                f"{feedback.distance_remaining:.2f} m, "
                f"recoveries={feedback.number_of_recoveries}"
            )

        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()

        completed_mode = self.navigation_mode

        self.goal_in_progress = False
        self.navigation_mode = None

        self.latest_forward_goal = None
        self.last_sent_goal = None

        if result == TaskResult.SUCCEEDED:

            if completed_mode == "normal":
                self.get_logger().info(
                    "Normal waypoint reached."
                )

                self.enter_recovery_mode = False

            elif completed_mode == "recovery":
                self.get_logger().warn(
                    "Recovery movement succeeded. "
                    "Resuming normal navigation."
                )

                self.enter_recovery_mode = False

            return

        # ------------------------------------------------------
        # Navigation failed
        # ------------------------------------------------------

        if completed_mode == "normal":
            self.get_logger().error(
                f"Normal navigation failed. Result: {result}. "
                f"Entering recovery mode."
            )

            self.enter_recovery_mode = True
            self.robot_recovery()

        elif completed_mode == "recovery":
            self.get_logger().error(
                f"Recovery navigation failed. Result: {result}."
            )

            self.enter_recovery_mode = True
            self.schedule_recovery_retry()

    def schedule_recovery_retry(self):
        if hasattr(self, "recovery_retry_timer"):
            return

        self.get_logger().warn(
            "Retrying recovery search in 1 second."
        )

        self.recovery_retry_timer = self.create_timer(
            1.0,
            self.retry_recovery_goal
        )

    def send_far_goal(self, goal_pose: PoseStamped):
        """
        Send a goal that is far away from the current position.
        Only used when the robot is completely stuck and needs to move to a distant location.
        """

        goal_pose.header.frame_id = self.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0


        
        pass
    
    # ======================================================
    # Reset the timer for when no valid candidates are found
    # ======================================================
    def reset_no_candidate_timer(self):
        self.no_candidate_since = self.get_clock().now()

    def handle_no_candidate_timeout(self):
        if self.no_candidate_since is None:
            return

        elapsed_time = (
            self.get_clock().now() - self.no_candidate_since
        ).nanoseconds / 1e9  # Convert to seconds

        if elapsed_time >= self.no_candidate_timeout:
            self.get_logger().warn(
                "No valid candidates found for an extended period. "
                "Entering recovery mode."
            )
            self.enter_recovery_mode = True
            self.no_candidate_since = None  # Reset the timer

    
    def no_candidate_found_recovery_waypoint(self):
        # Getting the dimensions of the map
        width = self.global_costmap_data.info.width
        height = self.global_costmap_data.info.height
        resolution = self.global_costmap_data.info.resolution
        origin_x = self.global_costmap_data.info.origin.position.x
        origin_y = self.global_costmap_data.info.origin.position.y
        grid = np.array(self.map_data.data).reshape((height, width))

        free_indices = np.argwhere(grid == 0)

        if free_indices.size == 0:
            self.get_logger().error("No free space found in the map for recovery.")
            return None
        
        # Randomly select a free cell in the forward half of the map

        

    def global_costmap_callback(self, msg: OccupancyGrid):
        # Accessing map data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Acquiring the map data
        self.global_costmap_data = msg.data
   	
    

if __name__ == '__main__':
    # import rclpy
    # from rclpy.node import Node

    rclpy.init()
    navigator = WaypointNavigator()

    # navigator.send_goal(navigator.goal_pose)  # Send the goal to the navigation stack
    rclpy.spin(navigator)
    rclpy.destroy_node(navigator)
    rclpy.shutdown()
