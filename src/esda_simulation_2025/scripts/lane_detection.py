#!/usr/bin/env python3
"""
Lane Detection Node for ESDA Simulation
Detects white lane markings from camera feed and publishes them as obstacles to SLAM map
"""

# from build.esda_simulation_2025.rosidl_generator_py.esda_simulation_2025.msg._navigation_recommendation import NavigationRecommendation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField, LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point


# ==========================================================================
# Pure helpers - no ROS state, so tools/validate_waypoint_logic.py can test
# them offline.
# ==========================================================================

def transform_to_matrix(transform):
    """
    Rotation (3x3) and translation (3,) of a geometry_msgs TransformStamped,
    so a point cloud can be transformed in one numpy operation instead of one
    do_transform_point call per point.
    """
    q = transform.transform.rotation
    t = transform.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w

    rotation = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])

    return rotation, np.array([t.x, t.y, t.z])


def inject_lane_ranges(ranges, angle_min, angle_max, angle_increment, range_max, lx, ly):
    """
    Write lane points (laser-frame x/y) into a LaserScan's ranges.

    Same rules as the original per-point loop: a point must lie inside the
    scan's angular span, within 10 m on each axis, 0.15 m to range_max from
    the laser; it replaces its beam if that beam is inf, NaN or 0.0 (no
    return) or further away. With several points on one beam the closest
    wins, exactly as the sequential loop produced.

    Returns (ranges as float64 array, number of beams changed).
    """
    ranges = np.array(ranges, dtype=np.float64)
    lx = np.asarray(lx, dtype=np.float64)
    ly = np.asarray(ly, dtype=np.float64)

    dist = np.sqrt(lx * lx + ly * ly)
    angle = np.arctan2(ly, lx)

    keep = (
        (angle >= angle_min) & (angle <= angle_max)
        & (np.abs(lx) < 10.0) & (np.abs(ly) < 10.0)
        & (dist >= 0.15) & (dist <= range_max)
    )

    # angle >= angle_min here, so truncation equals the loop's int().
    idx = ((angle[keep] - angle_min) / angle_increment).astype(np.int64)
    dist = dist[keep]

    in_scan = (idx >= 0) & (idx < ranges.size)
    idx = idx[in_scan]
    dist = dist[in_scan]

    lane_min = np.full(ranges.size, np.inf)
    np.minimum.at(lane_min, idx, dist)

    hit = np.isfinite(lane_min)
    current = ranges[hit]
    replace = ~np.isfinite(current) | (current == 0.0) | (lane_min[hit] < current)

    ranges[hit] = np.where(replace, lane_min[hit], current)

    return ranges, int(np.count_nonzero(replace))


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        # Parameters
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('white_threshold_low', 130)  # Grayscale threshold for white (more lenient)
        self.declare_parameter('white_threshold_high', 255)
        self.declare_parameter('min_line_length', 30)  # Shorter to detect more lanes
        self.declare_parameter('max_line_gap', 40)  # Larger gap tolerance
        self.declare_parameter('min_lane_width', 15)  # Minimum lane width in pixels
        self.declare_parameter('max_lane_width', 200)  # Maximum lane width in pixels
        self.declare_parameter('lane_thickness_pixels', 8)  # Lane thickness for dense sampling
        self.declare_parameter('point_spacing_pixels', 2.0)  # Distance between sampled points

        self.declare_parameter('virtual_lane_length', 3.0)
        self.declare_parameter('virtual_lane_spacing', 0.05)
        self.declare_parameter('virtual_lane_fit_points', 8)

        # Lane points older than this are not injected into /scan_fused. They
        # are in the camera frame, so stale ones ride along with the robot.
        self.declare_parameter('lane_points_max_age', 0.5)

        # Ground-plane fallback geometry. Height is above the GROUND, not
        # base_link: base_link sits at axle height (wheel_radius 0.1625 in
        # robot_core_ref.xacro) and camera_joint adds 0.315 (camera.xacro).
        self.declare_parameter('camera_height', 0.4775)
        self.declare_parameter('camera_pitch', 0.0)  # radians, 0 = looking straight ahead

        # World-fixed frame lane points are anchored in between the image and
        # the scans they are injected into, so robot motion in that gap does
        # not smear them. odom = EKF world_frame / diff-drive odom_frame_id.
        self.declare_parameter('lane_fixed_frame', 'odom')

        # Get parameters
        self.show_viz = self.get_parameter('show_visualization').value
        self.white_low = self.get_parameter('white_threshold_low').value
        self.white_high = self.get_parameter('white_threshold_high').value
        self.min_line_length = self.get_parameter('min_line_length').value
        self.max_line_gap = self.get_parameter('max_line_gap').value
        self.min_lane_width = self.get_parameter('min_lane_width').value
        self.max_lane_width = self.get_parameter('max_lane_width').value
        self.lane_thickness = self.get_parameter('lane_thickness_pixels').value
        self.point_spacing = self.get_parameter('point_spacing_pixels').value

        self.virtual_lane_length = self.get_parameter(
            'virtual_lane_length'
        ).value

        self.virtual_lane_spacing = self.get_parameter(
            'virtual_lane_spacing'
        ).value

        self.virtual_lane_fit_points = self.get_parameter(
            'virtual_lane_fit_points'
        ).value
        
        # CV Bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()
        
        # QoS profile for all topics (matches ros_gz_bridge RELIABLE)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber to camera topic (using Left camera for processing)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/left/image_raw',
            self.image_callback,
            reliable_qos
        )

        # Subscriber to Right camera (for stereo visualization only)
        self.right_image_sub = self.create_subscription(
            Image,
            '/camera/right/image_raw',
            self.right_image_callback,
            reliable_qos
        )

        # Subscriber to Depth Image
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            reliable_qos
        )
        
        # Publisher for lane markers (visualized is markers in RViz)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/lane_markers',
            10
        )

        # Publisher for PointCloud for Nav2 costmap
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/lane_obstacles',
            10
        )

        # self.behaviour_tree_publisher = self.create_publisher(
        #     NavigationRecommendation,
        #     '/behaviour_tree', 
        #     10
        # )

        # Publisher for extrapolated / virtual lane boundaries (if needed)
        self.virtual_lane_pub = self.create_publisher(
            PointCloud2,
            '/virtual_lane_points',
            10
        )

        self.virtual_lane_marker_pub = self.create_publisher(
            MarkerArray,
            '/virtual_lane_markers',
            10
        )

        # TF Buffer for transforming points
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Lidar Interaction (scan also uses RELIABLE from gz_bridge)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, reliable_qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_fused', reliable_qos)
        
        # Store latest processed image for visualization
        self.latest_viz_image = None
        self.latest_depth_image = None
        self.latest_right_image = None
        self.latest_lines = []
        self.latest_3d_points = [] # Store detected points in camera frame
        self.latest_3d_points_time = None
        self.lane_points_max_age = self.get_parameter('lane_points_max_age').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_pitch = self.get_parameter('camera_pitch').value
        self.lane_fixed_frame = self.get_parameter('lane_fixed_frame').value
        self.latest_lane_points_fixed = None  # N x 3, in lane_fixed_frame

        self.left_lane_points = []
        self.right_lane_points = []
        
        self.get_logger().info('Lane Detection Node initialized (Stereo Mode)')
        self.get_logger().info(f'Subscribing to: /camera/left/image_raw, /camera/right/image_raw, /camera/depth/image_raw')
        self.get_logger().info(f'Publishing to: /lane_markers, /lane_obstacles, /scan_fused')
        self.get_logger().info(f'Show visualization: {self.show_viz}')
    
    def right_image_callback(self, msg):
        """Store the latest right image for stereo visualization"""
        try:
            self.latest_right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Right Image Error: {e}')

    def scan_callback(self, msg):
        """
        Receive LaserScan, inject lane obstacles, and republish to /scan_fused
        """
        # If no lanes detected, just republish the original scan
        if not self.latest_3d_points or self.latest_3d_points_time is None:
            self.scan_pub.publish(msg)
            return

        # Also skip points the image pipeline has stopped refreshing (e.g. a
        # processing exception), rather than injecting them indefinitely.
        points_age = (self.get_clock().now() - self.latest_3d_points_time).nanoseconds / 1e9
        if points_age > self.lane_points_max_age:
            self.scan_pub.publish(msg)
            return

        laser_frame = msg.header.frame_id

        if self.latest_lane_points_fixed is None:
            self.scan_pub.publish(msg)
            return

        # Lane points are held in lane_fixed_frame, anchored at the image
        # stamp. Bring them into the laser frame at THIS scan's stamp so robot
        # motion since the image was taken does not smear them.
        transform = self.lookup_transform_at(
            laser_frame, self.lane_fixed_frame, msg.header.stamp)
        if transform is None:
            self.scan_pub.publish(msg)
            return

        rotation, translation = transform_to_matrix(transform)
        points_laser = self.latest_lane_points_fixed @ rotation.T + translation

        ranges, injected_count = inject_lane_ranges(
            msg.ranges, msg.angle_min, msg.angle_max, msg.angle_increment,
            msg.range_max, points_laser[:, 0], points_laser[:, 1])

        # tolist() gives native Python floats; rclpy rejects numpy scalars.
        msg.ranges = ranges.tolist()
        self.scan_pub.publish(msg)

        if injected_count > 0:
            self.get_logger().info(
                f'Injected lane points into {injected_count} beams (frame: {laser_frame})',
                throttle_duration_sec=2.0)

    def publish_virtual_lane_markers(self, virtual_left, virtual_right):
        """
        Publish virtual lane points as orange RViz markers.
        """

        marker_array = MarkerArray()

        marker_id = 0

        for points in [virtual_left, virtual_right]:

            for x, y, z in points:

                marker = Marker()

                marker.header.frame_id = 'base_link'
                marker.header.stamp = self.get_clock().now().to_msg()

                marker.ns = 'virtual_lanes'
                marker.id = marker_id

                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = float(x)
                marker.pose.position.y = float(y)
                marker.pose.position.z = float(z + 0.03)

                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.08
                marker.scale.y = 0.08
                marker.scale.z = 0.08

                # Orange
                marker.color.r = 1.0
                marker.color.g = 0.55
                marker.color.b = 0.0
                marker.color.a = 1.0

                # Short lifetime so old predictions disappear
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 300000000

                marker_array.markers.append(marker)

                marker_id += 1

        self.virtual_lane_marker_pub.publish(marker_array)

    def lookup_transform_at(self, target_frame, source_frame, stamp_msg):
        """
        Transform at a message's stamp, falling back to the latest available
        one (e.g. the stamp is just ahead of the newest odom TF). Returns None
        if neither is available.
        """
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time.from_msg(stamp_msg))
        except TransformException as ex:
            self.get_logger().warn(
                f'No {source_frame} -> {target_frame} transform at message stamp, '
                f'using latest (lane motion compensation reduced): {ex}',
                throttle_duration_sec=10.0)

        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {source_frame} -> {target_frame}: {ex}',
                throttle_duration_sec=5.0)
            return None

    def depth_callback(self, msg):
        """Store the latest depth image"""
        try:
            # Depth image is usually 16UC1 (uint16 in mm) or 32FC1 (float in meters)
            # The libgazebo_ros_camera plugin usually sends 32FC1
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.get_logger().info(f'Depth image received: shape={self.latest_depth_image.shape}, '
                                   f'min={np.nanmin(self.latest_depth_image):.2f}, '
                                   f'max={np.nanmax(self.latest_depth_image):.2f}', 
                                   throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f'Depth Callback Error: {e}')
        
    def detect_white_lanes(self, image):
        """
        Detect white lane markings by finding white lines with dark tarmac on both sides
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        
        # Restrict ROI to lower half of the image
        roi_mask = np.zeros_like(gray)
        roi_vertices = np.array([[
            (0, height),
            (0, int(height * 0.5)),
            (width, int(height * 0.5)),
            (width, height)
        ]], dtype=np.int32)
        cv2.fillPoly(roi_mask, roi_vertices, 255)
        gray_roi = cv2.bitwise_and(gray, roi_mask)
        
        # Threshold for white colors
        # Balance noise and detection - use parameter value
        _, white_mask = cv2.threshold(gray_roi, self.white_low, 255, cv2.THRESH_BINARY)
        
        # Apply morphological operations
        # Use 3x3 kernel for opening to preserve thin lane lines while removing tiny sparkles
        kernel = np.ones((3, 3), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        
        # Edge detection
        edges = cv2.Canny(white_mask, 50, 150)
        
        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=15,  # Lower threshold for more detections
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        # Filter lines to find valid white strips (Dark-White-Dark)
        valid_lines = self.filter_valid_lanes(lines, gray, height, width) if lines is not None else []
        
        return valid_lines, white_mask, edges
    
    def filter_valid_lanes(self, lines, gray, height, width):
        """
        Filter for lines that look like lane markings (white on dark)
        """
        valid_lines = []
        if lines is None:
            return []
            
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate line length
            length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
            if length < self.min_line_length:
                continue
                
            # Calculate angle (reject very horizontal lines, but be lenient)
            angle = np.abs(np.arctan2(y2-y1, x2-x1))
            if angle < 0.05: # Too horizontal
                continue
                
            # Verify Profile: Dark - White - Dark
            if self.verify_line_profile(gray, x1, y1, x2, y2):
                valid_lines.append(line[0])
                
        return valid_lines
    
    def verify_line_profile(self, gray, x1, y1, x2, y2):
        """
        Check if the line sits on a white strip surrounded by dark
        """
        # Midpoint
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        
        # Perpendicular direction
        dx, dy = x2 - x1, y2 - y1
        mag = np.sqrt(dx*dx + dy*dy)
        if mag == 0: return False
        dx, dy = dx/mag, dy/mag
        
        # Normal vector (rotate 90 deg)
        nx, ny = -dy, dx
        
        # Check profile at midpoint
        # Check center (should be white)
        if not self.is_pixel_white(gray, mx, my):
            return False
            
        # Check sides (should be dark)
        # Check at distance ~10-12 pixels away (slightly shorter for more detection)
        check_dist = 12
        p1x, p1y = mx + nx * check_dist, my + ny * check_dist
        p2x, p2y = mx - nx * check_dist, my - ny * check_dist
        
        is_dark_1 = self.is_pixel_dark(gray, p1x, p1y)
        is_dark_2 = self.is_pixel_dark(gray, p2x, p2y)
        
        # Accept if at least one side is dark (less strict)
        return is_dark_1 or is_dark_2

    def is_pixel_white(self, img, x, y):
        h, w = img.shape
        x, y = int(x), int(y)
        if 0 <= y < h and 0 <= x < w:
            return img[y, x] > 120  # More lenient threshold
        return False

    def is_pixel_dark(self, img, x, y):
        h, w = img.shape
        x, y = int(x), int(y)
        if 0 <= y < h and 0 <= x < w:
            return img[y, x] < 110  # More lenient threshold
        return True # Assume dark if out of bounds (safe)
    
    def image_callback(self, msg):
        """
        Process incoming camera images
        """
        self.get_logger().info('Image callback received', throttle_duration_sec=5.0)
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect lanes
            lines, white_mask, edges = self.detect_white_lanes(cv_image)
            
            # Create visualization image
            viz_image = cv_image.copy()
            detection_image = np.zeros_like(cv_image)
            
            # Store detected lines
            self.latest_lines = []
            
            if lines and len(lines) > 0:
                for line in lines:
                    x1, y1, x2, y2 = line
                    # Draw on visualization
                    cv2.line(viz_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    # Draw on detection overlay
                    cv2.line(detection_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Store line for marker publishing
                    self.latest_lines.append(line)
                
                self.get_logger().info(f'Detected {len(lines)} lane segments', throttle_duration_sec=2.0)
            
            # Create composite visualization
            if self.show_viz:
                # Show white mask
                white_mask_colored = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)
                
                # Show edges
                edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                
                # Create Stereo Top Row (Left and Right images)
                left_view = viz_image.copy()
                right_view = self.latest_right_image.copy() if self.latest_right_image is not None else np.zeros_like(cv_image)
                
                stereo_row = np.hstack([left_view, right_view])
                
                # Create Processing Bottom Row (Mask and Edges)
                processing_row = np.hstack([white_mask_colored, edges_colored])
                
                # Combine rows
                combined = np.vstack([stereo_row, processing_row])
                
                # Resize for easier viewing
                combined_resized = cv2.resize(combined, (1280, 720))
                
                # Add labels
                cv2.putText(combined_resized, 'LIVE STEREO: LEFT', (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(combined_resized, 'LIVE STEREO: RIGHT', (650, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(combined_resized, 'COMPUTER VISION: WHITE MASK', (10, 390),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(combined_resized, f'DETECTED SEGMENTS: {len(lines) if lines is not None else 0}',
                           (650, 390),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow('Lane Detection - ESDA STEREO', combined_resized)
                cv2.waitKey(1)
            
            # Publish markers for detected lanes
            if lines and len(self.latest_lines) > 0:
                # Convert to format expected by publish_lane_markers
                lines_array = [[line] for line in self.latest_lines]
                header = msg.header
                header.frame_id = 'camera_link_optical' # Use the optical frame for projection
                
                self.publish_lane_markers(lines_array, header)

                # publish_lane_markers() has now populated:
                #
                # self.left_lane_points
                # self.right_lane_points
                #
                # Use these to create predicted lane boundaries.
                self.publish_virtual_lane_cloud()

                # Also publish the REAL detected lane points
                if self.latest_depth_image is not None:
                    self.publish_obstacle_cloud(
                        self.latest_lines,
                        self.latest_depth_image,
                        header
                    )
                else:
                    self.latest_3d_points = []
            else:
                # No lanes this frame: drop the previous detection so
                # scan_callback stops injecting it into /scan_fused.
                self.latest_3d_points = []
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
            
    def publish_obstacle_cloud(self, lines, depth_img, header):
        """
        Convert detected lane pixels to 3D points using Depth image and publish as PointCloud2
        Creates thick lane lines by sampling perpendicular to line direction
        """
        points = []
        self.latest_3d_points = []
        
        # Camera Intrinsics (Approximate for 640x480, FOV ~1.089 rad)
        # fx = width / (2 * tan(fov/2))
        fx = 640 / (2 * np.tan(1.089 / 2))
        fy = fx
        cx = 320
        cy = 240
        
        # Camera height and tilt for ground plane fallback
        camera_height = self.camera_height
        camera_pitch = self.camera_pitch
        
        valid_depth_count = 0
        fallback_count = 0
        
        for line in lines:
            x1, y1, x2, y2 = line
            
            # Calculate line direction and perpendicular
            dx = x2 - x1
            dy = y2 - y1
            line_len = np.sqrt(dx**2 + dy**2)
            
            if line_len < 1.0:
                continue
                
            # Normalize
            dx_norm = dx / line_len
            dy_norm = dy / line_len
            
            # Perpendicular direction (for thickness)
            perp_x = -dy_norm
            perp_y = dx_norm
            
            # Sample points along the line at regular intervals
            num_samples_along = max(int(line_len / self.point_spacing), 3)
            # Sample across the line thickness
            num_samples_across = max(int(self.lane_thickness / 2), 3)
            
            # Sample along and across the line to create thick lanes
            for t_along in np.linspace(0, 1, num_samples_along):
                # Center point along line
                center_u = x1 + t_along * dx
                center_v = y1 + t_along * dy
                
                # Sample across the thickness
                for t_across in np.linspace(-self.lane_thickness/2, self.lane_thickness/2, num_samples_across):
                    u = int(center_u + t_across * perp_x)
                    v = int(center_v + t_across * perp_y)
                    
                    if not (0 <= u < 640 and 0 <= v < 480):
                        continue
                        
                    d = depth_img[v, u]
                    
                    # Convert to native Python float to handle numpy types
                    d = float(d)
                    
                    # Check if depth is valid
                    use_fallback = False
                    if math.isnan(d) or math.isinf(d) or d <= 0.05 or d > 10.0:
                        use_fallback = True
                    
                    # If depth is invalid, estimate using ground plane assumption
                    if use_fallback:
                        # Ray direction in camera frame (optical: +Z forward, +X right, +Y down)
                        ray_x = (u - cx) / fx
                        ray_y = (v - cy) / fy
                        ray_z = 1.0
                        
                        # Normalize
                        ray_len = math.sqrt(ray_x**2 + ray_y**2 + ray_z**2)
                        ray_x /= ray_len
                        ray_y /= ray_len  
                        ray_z /= ray_len
                        
                        # Apply camera pitch (if any)
                        # Rotate ray around X axis by pitch
                        ray_y_rot = ray_y * math.cos(camera_pitch) - ray_z * math.sin(camera_pitch)
                        ray_z_rot = ray_y * math.sin(camera_pitch) + ray_z * math.cos(camera_pitch)
                        
                        # Intersect with ground plane
                        # Camera optical frame: Y is down, so ground is at y = camera_height
                        # Point on ray: (ray_x * t, ray_y_rot * t, ray_z_rot * t)
                        # Ground: y = camera_height
                        # Solve: ray_y_rot * t = camera_height
                        if abs(ray_y_rot) > 0.01:  # Ray not parallel to ground
                            t_intersect = camera_height / ray_y_rot
                            if 0.3 < t_intersect < 8.0:  # Reasonable distance range
                                z = t_intersect * ray_z_rot
                                x = t_intersect * ray_x
                                y = t_intersect * ray_y_rot
                                
                                # Additional validation: check if point is in front of camera
                                if z > 0.2 and abs(x) < 5.0:
                                    fallback_count += 1
                                else:
                                    continue
                            else:
                                continue
                        else:
                            continue
                    else:
                        # Use actual depth data
                        z = d
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        
                        # Validate the 3D point
                        if z < 0.2 or z > 8.0 or abs(x) > 5.0:
                            continue
                            
                        valid_depth_count += 1
                    
                    # Add to points list (as native Python floats)
                    points.append([float(x), float(y), float(z)])
                    self.latest_3d_points.append((float(x), float(y), float(z)))
        
        self.get_logger().info(f'Generated {len(points)} 3D points from {len(lines)} lane segments '\
                               f'(valid_depth={valid_depth_count}, fallback={fallback_count})', 
                               throttle_duration_sec=2.0)
        
        self.latest_3d_points_time = self.get_clock().now()

        # Anchor the points in a world-fixed frame at the IMAGE stamp, so
        # scan_callback can place them correctly however far the robot has
        # moved by the time each later scan arrives.
        self.latest_lane_points_fixed = None
        if points:
            transform = self.lookup_transform_at(
                self.lane_fixed_frame, 'camera_link_optical', header.stamp)
            if transform is not None:
                rotation, translation = transform_to_matrix(transform)
                self.latest_lane_points_fixed = (
                    np.asarray(points, dtype=np.float64) @ rotation.T + translation
                )

        if not points:
            self.get_logger().warn('No valid 3D points generated from lanes', throttle_duration_sec=5.0)
            return

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header # Use same header/frame as camera
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.is_dense = False
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        
        # Pack binary data
        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', p[0], p[1], p[2]))
            
        msg.data = b''.join(buffer)
        
        self.cloud_pub.publish(msg)

    def publish_lane_markers(self, lines, header):
        """
        Publish detected lanes as markers showing actual lane segments at ground level.
        Also split sampled points into left and right lane lists.
        """
        marker_array = MarkerArray()

        # Store usable lane data for control
        self.left_lane_points = []
        self.right_lane_points = []

        # Camera intrinsics
        fx = 640 / (2 * np.tan(1.089 / 2))
        fy = fx
        cx = 320
        cy = 240
        camera_height = self.camera_height
        camera_pitch = self.camera_pitch

        marker_id = 0

        for line in lines:
            line = line[0]   # Unwrap
            x1, y1, x2, y2 = line

            # Decide whether this line is on the left or right side of the image
            u_mid = 0.5 * (x1 + x2)
            is_left_lane = u_mid < cx

            # Calculate line parameters
            dx = x2 - x1
            dy = y2 - y1
            line_len = np.sqrt(dx**2 + dy**2)

            if line_len < 1.0:
                continue

            num_markers = max(int(line_len / 5.0), 2)

            for t in np.linspace(0, 1, num_markers):
                u = int(x1 + t * dx)
                v = int(y1 + t * dy)

                if not (0 <= u < 640 and 0 <= v < 480):
                    continue

                x_3d, y_3d, z_3d = None, None, None

                if self.latest_depth_image is not None:
                    d = self.latest_depth_image[v, u]
                    if not np.isnan(d) and not np.isinf(d) and 0.1 < d < 10.0:
                        z_3d = float(d)
                        x_3d = float((u - cx) * z_3d / fx)
                        y_3d = float((v - cy) * z_3d / fy)

                if x_3d is None:
                    ray_x = (u - cx) / fx
                    ray_y = (v - cy) / fy
                    ray_z = 1.0

                    ray_len = math.sqrt(ray_x**2 + ray_y**2 + ray_z**2)
                    ray_x /= ray_len
                    ray_y /= ray_len
                    ray_z /= ray_len

                    ray_y_rot = ray_y * math.cos(camera_pitch) - ray_z * math.sin(camera_pitch)
                    ray_z_rot = ray_y * math.sin(camera_pitch) + ray_z * math.cos(camera_pitch)

                    if abs(ray_y_rot) > 0.01:
                        t_intersect = camera_height / ray_y_rot
                        if 0.3 < t_intersect < 8.0:
                            z_3d = float(t_intersect * ray_z_rot)
                            x_3d = float(t_intersect * ray_x)
                            y_3d = float(t_intersect * ray_y_rot)

                if x_3d is None:
                    continue

                # Store point for control logic
                point = (x_3d, y_3d, z_3d)
                if is_left_lane:
                    self.left_lane_points.append(point)
                else:
                    self.right_lane_points.append(point)

                marker = Marker()
                marker.header = header
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                if is_left_lane:
                    marker.ns = 'left_lane'
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    marker.ns = 'right_lane'
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0

                marker.pose.position.x = x_3d
                marker.pose.position.y = y_3d
                marker.pose.position.z = z_3d
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.08
                marker.scale.y = 0.08
                marker.scale.z = 0.05
                marker.color.a = 0.9

                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 500000000

                marker_array.markers.append(marker)
                marker_id += 1

        self.marker_pub.publish(marker_array)
    
    # Deprecated methods removed (image_y_to_distance, etc)

    def transform_lane_points_to_base(self, points):
        """
        Transform lane points from camera_link_optical into base_link.

        Returns:
            List of (x, y, z) tuples in base_link.
        """

        if not points:
            return []

        transformed_points = []

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link_optical',
                rclpy.time.Time()
            )

            for point in points:
                p = PointStamped()
                p.header.frame_id = 'camera_link_optical'

                p.point.x = float(point[0])
                p.point.y = float(point[1])
                p.point.z = float(point[2])

                p_base = do_transform_point(p, transform)

                transformed_points.append((
                    float(p_base.point.x),
                    float(p_base.point.y),
                    float(p_base.point.z)
                ))

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform lane points to base_link: {ex}',
                throttle_duration_sec=2.0
            )

            return []

        return transformed_points

    def extrapolate_lane(self, lane_points):
        """
        Fit a local straight line to the furthest visible part of a lane and
        extrapolate it forward.

        lane_points must already be in base_link.

        Returns:
            List of virtual (x, y, z) points in base_link.
        """

        if len(lane_points) < 3:
            return []

        # Remove points behind the robot
        forward_points = [
            p for p in lane_points
            if p[0] > 0.0
        ]

        if len(forward_points) < 3:
            return []

        # Sort by forward distance
        forward_points.sort(key=lambda p: p[0])

        # Only use the furthest few points to estimate the direction
        fit_count = min(
            self.virtual_lane_fit_points,
            len(forward_points)
        )

        fit_points = forward_points[-fit_count:]

        x_vals = np.array(
            [p[0] for p in fit_points],
            dtype=np.float64
        )

        y_vals = np.array(
            [p[1] for p in fit_points],
            dtype=np.float64
        )

        z_vals = np.array(
            [p[2] for p in fit_points],
            dtype=np.float64
        )

        # Need some difference in x or the fit becomes unstable
        if np.ptp(x_vals) < 0.05:
            return []

        # Fit:
        #
        # y = m*x + b
        #
        try:
            m, b = np.polyfit(x_vals, y_vals, 1)
        except Exception as ex:
            self.get_logger().warn(
                f'Virtual lane fit failed: {ex}',
                throttle_duration_sec=2.0
            )
            return []

        # Start prediction at the furthest observed point
        start_x = float(np.max(x_vals))

        end_x = start_x + self.virtual_lane_length

        average_z = float(np.mean(z_vals))

        virtual_points = []

        for x in np.arange(
            start_x,
            end_x,
            self.virtual_lane_spacing
        ):
            y = m * x + b

            virtual_points.append((
                float(x),
                float(y),
                average_z
            ))

        return virtual_points
    
    def publish_virtual_lane_cloud(self):
        """
        Extrapolate the currently observed left/right lanes and publish
        the resulting virtual boundaries as PointCloud2 in base_link.
        """

        self.get_logger().warn(
            'publish_virtual_lane_cloud() CALLED',
            throttle_duration_sec=1.0
        )

        left_base = self.transform_lane_points_to_base(
            self.left_lane_points
        )

        right_base = self.transform_lane_points_to_base(
            self.right_lane_points
        )

        self.get_logger().warn(
            f'raw_left={len(self.left_lane_points)}, '
            f'raw_right={len(self.right_lane_points)}, '
            f'base_left={len(left_base)}, '
            f'base_right={len(right_base)}',
            throttle_duration_sec=1.0
        )

        if left_base:
            self.get_logger().warn(
                f'LEFT SAMPLE: {left_base[:3]}',
                throttle_duration_sec=1.0
            )

        if right_base:
            self.get_logger().warn(
                f'RIGHT SAMPLE: {right_base[:3]}',
                throttle_duration_sec=1.0
            )


        virtual_left = self.extrapolate_lane(left_base)
        virtual_right = self.extrapolate_lane(right_base)

        self.publish_virtual_lane_markers(
            virtual_left,
            virtual_right
        )

        self.get_logger().warn(
            f'virtual_left={len(virtual_left)}, '
            f'virtual_right={len(virtual_right)}',
            throttle_duration_sec=1.0
        )


        virtual_points = virtual_left + virtual_right

        if not virtual_points:
            self.get_logger().warn(
                'No virtual points generated',
                throttle_duration_sec=1.0
            )
            return

        # Feed the extrapolated line into the same topic the local/global
        # costmaps already subscribe to for /lane_obstacles, so the virtual
        # boundary is actually treated as an obstacle by the local planner
        # (not just drawn in RViz).
        obstacle_msg = PointCloud2()
        obstacle_msg.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg.header.frame_id = 'base_link'
        obstacle_msg.height = 1
        obstacle_msg.width = len(virtual_points)
        obstacle_msg.is_bigendian = False
        obstacle_msg.is_dense = False
        obstacle_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        obstacle_msg.point_step = 12
        obstacle_msg.row_step = obstacle_msg.point_step * len(virtual_points)
        obstacle_msg.data = b''.join(
            struct.pack('fff', float(x), float(y), float(z))
            for x, y, z in virtual_points
        )
        self.cloud_pub.publish(obstacle_msg)

        msg = PointCloud2()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.height = 1
        msg.width = len(virtual_points)

        msg.is_bigendian = False
        msg.is_dense = True

        msg.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='rgb',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1
            ),
        ]

        # Orange: RGB(255, 140, 0)
        r = 255
        g = 140
        b = 0

        rgb_uint32 = (r << 16) | (g << 8) | b

        rgb_float = struct.unpack(
            'f',
            struct.pack('I', rgb_uint32)
        )[0]

        msg.point_step = 16
        msg.row_step = msg.point_step * len(virtual_points)

        # msg.point_step = 12
        # msg.row_step = msg.point_step * len(virtual_points)

        buffer = []

        for x, y, z in virtual_points:
            buffer.append(
                struct.pack(
                    'ffff',
                    float(x),
                    float(y),
                    float(z),
                    rgb_float
                )
            )
        msg.data = b''.join(buffer)

        self.virtual_lane_pub.publish(msg)

        self.get_logger().info(
            f'Published {len(virtual_points)} virtual lane points '
            f'(left={len(virtual_left)}, '
            f'right={len(virtual_right)})',
            throttle_duration_sec=2.0
        )

def main(args=None):
    rclpy.init(args=args)
    
    node = LaneDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()