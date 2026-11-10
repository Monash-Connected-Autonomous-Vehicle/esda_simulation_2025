#!/usr/bin/env python3
"""
Lane Detection Node for ESDA Simulation
Detects white lane markings from camera feed and publishes them as obstacles to SLAM map
"""

# from build.esda_simulation_2025.rosidl_generator_py.esda_simulation_2025.msg._navigation_recommendation import NavigationRecommendation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField, LaserScan, CameraInfo
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

# encoding -> (dtype, channels)
_IMAGE_ENCODINGS = {
    'bgr8': (np.uint8, 3), 'rgb8': (np.uint8, 3), '8UC3': (np.uint8, 3),
    'bgra8': (np.uint8, 4), 'rgba8': (np.uint8, 4), '8UC4': (np.uint8, 4),
    'mono8': (np.uint8, 1), '8UC1': (np.uint8, 1),
    'mono16': (np.uint16, 1), '16UC1': (np.uint16, 1),
    '32FC1': (np.float32, 1),
}
_TO_BGR = {'rgb8': cv2.COLOR_RGB2BGR, 'bgra8': cv2.COLOR_BGRA2BGR, '8UC4': cv2.COLOR_BGRA2BGR,
           'rgba8': cv2.COLOR_RGBA2BGR, 'mono8': cv2.COLOR_GRAY2BGR, '8UC1': cv2.COLOR_GRAY2BGR}


def image_msg_to_cv(msg, desired_encoding='passthrough'):
    """sensor_msgs/Image -> numpy array, without cv_bridge.

    Humble's cv_bridge is built against NumPy 1.x and segfaults on real
    images under the NumPy 2 in ~/.local, so convert by hand. Handles the
    simulator's bgr8/32FC1 and the ZED's bgra8.
    """
    dtype, channels = _IMAGE_ENCODINGS[msg.encoding]
    itemsize = np.dtype(dtype).itemsize
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * channels * itemsize].copy().view(dtype)
    if msg.is_bigendian and itemsize > 1:
        img = img.byteswap()
    img = img.reshape(msg.height, msg.width, channels) if channels > 1 else img.reshape(msg.height, msg.width)
    if desired_encoding == 'bgr8' and msg.encoding not in ('bgr8', '8UC3'):
        img = cv2.cvtColor(img, _TO_BGR[msg.encoding])
    return img


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

        # Cone rejection: a cone's white reflective stripe is genuinely white
        # (grayscale/dark-white-dark checks alone can't tell it apart from
        # paint), but it always sits directly against the cone's orange body.
        # Reject any candidate line with a significant patch of saturated
        # orange nearby - real lane paint never borders orange.
        self.declare_parameter('reject_orange_cones', True)
        self.declare_parameter('orange_hue_low', 5)      # OpenCV HSV hue is 0-179
        self.declare_parameter('orange_hue_high', 25)
        self.declare_parameter('orange_sat_min', 100)    # 0-255; cone orange is highly saturated
        self.declare_parameter('orange_val_min', 70)     # 0-255; excludes near-black false matches
        self.declare_parameter('orange_check_radius', 10)       # pixels; where the perpendicular search starts
        self.declare_parameter('orange_search_max_radius', 80)  # pixels; where it gives up

        # Which physical camera this instance is processing - lets multiple
        # instances (front ZED vs. remapped side cameras) each stamp their
        # output with their own TF frame and ground-plane geometry instead of
        # all sharing the front camera's values.
        self.declare_parameter('camera_frame_id', 'camera_link_optical')
        self.declare_parameter('camera_mount_height', 0.315)  # meters above ground
        self.declare_parameter('camera_mount_pitch', 0.0)  # radians (0 = looking straight ahead)
        # Hard cutoff on published point distance - projection noise grows with
        # range, so a tighter value here biases output toward closer, more
        # trustworthy detections instead of noisy far ones.
        self.declare_parameter('max_lane_range', 8.0)
        # Real cameras. camera_info_topic: take intrinsics and image size from
        # CameraInfo instead of the simulator's fixed 640x480 model.
        # ground_frame/ground_z: where depth is missing, intersect each pixel's
        # ray with the plane z = ground_z in ground_frame (looked up through
        # TF), instead of using camera_mount_height/pitch. Left empty, the
        # simulator behaviour is unchanged.
        self.declare_parameter('camera_info_topic', '')
        self.declare_parameter('ground_frame', '')
        self.declare_parameter('ground_z', 0.0)

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
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.camera_mount_height = self.get_parameter('camera_mount_height').value
        self.camera_mount_pitch = self.get_parameter('camera_mount_pitch').value
        self.max_lane_range = self.get_parameter('max_lane_range').value
        self.ground_frame = self.get_parameter('ground_frame').value
        self.ground_z = float(self.get_parameter('ground_z').value)
        self.ground_tf = None  # (R, t): camera frame -> ground_frame, refreshed per image

        # Simulator intrinsics (640x480, FOV ~1.089 rad) until CameraInfo says otherwise
        self.fx = 640 / (2 * np.tan(1.089 / 2))
        self.fy = self.fx
        self.cx = 320.0
        self.cy = 240.0
        self.img_w, self.img_h = 640, 480
        self.reject_orange_cones = self.get_parameter('reject_orange_cones').value
        self.orange_hue_low = self.get_parameter('orange_hue_low').value
        self.orange_hue_high = self.get_parameter('orange_hue_high').value
        self.orange_sat_min = self.get_parameter('orange_sat_min').value
        self.orange_val_min = self.get_parameter('orange_val_min').value
        self.orange_check_radius = self.get_parameter('orange_check_radius').value
        self.orange_search_max_radius = self.get_parameter('orange_search_max_radius').value
        
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

        # TF Buffer for transforming points
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Lidar Interaction (scan also uses RELIABLE from gz_bridge)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, reliable_qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_fused', reliable_qos)

        info_topic = self.get_parameter('camera_info_topic').value
        if info_topic:
            self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, reliable_qos)
        
        # Store latest processed image for visualization
        self.latest_viz_image = None
        self.latest_depth_image = None
        self.latest_right_image = None
        self.latest_lines = []
        self.latest_3d_points = [] # Store detected points in camera frame
        
        self.get_logger().info('Lane Detection Node initialized (Stereo Mode)')
        self.get_logger().info(f'Subscribing to: /camera/left/image_raw, /camera/right/image_raw, /camera/depth/image_raw')
        self.get_logger().info(f'Publishing to: /lane_markers, /lane_obstacles, /scan_fused')
        self.get_logger().info(f'Show visualization: {self.show_viz}')
    
    def camera_info_callback(self, msg):
        """Use the real camera's intrinsics and image size."""
        if msg.k[0] <= 0.0:
            return
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        self.img_w, self.img_h = msg.width, msg.height

    def update_ground_tf(self):
        """Cache the camera -> ground_frame transform for ground_point()."""
        self.ground_tf = None
        if not self.ground_frame:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.ground_frame, self.camera_frame_id, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'No transform {self.camera_frame_id} -> {self.ground_frame}: {ex}',
                throttle_duration_sec=5.0)
            return
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        t = tf.transform.translation
        self.ground_tf = (R, np.array([t.x, t.y, t.z]))

    def ground_point(self, u, v):
        """Camera-frame (x, y, z) where pixel (u, v)'s ray meets the ground, or None."""
        R, t = self.ground_tf
        ray = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        d = R @ ray
        if d[2] > -1e-3:  # at or above the horizon
            return None
        s = (self.ground_z - t[2]) / d[2]
        x, y, z = (float(c) for c in s * ray)
        if not (0.2 < z < 8.0 and abs(x) < 5.0):
            return None
        return x, y, z

    def right_image_callback(self, msg):
        """Store the latest right image for stereo visualization"""
        try:
            self.latest_right_image = image_msg_to_cv(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Right Image Error: {e}')

    def scan_callback(self, msg):
        """
        Receive LaserScan, inject lane obstacles, and republish to /scan_fused
        """
        # If no lanes detected, just republish the original scan
        if not self.latest_3d_points:
            self.scan_pub.publish(msg)
            return

        fused_scan = msg
        # Make ranges mutable
        ranges = list(fused_scan.ranges)
        
        # Transform cached 3D points to Laser Frame
        # Points are currently in 'camera_link_optical'
        camera_frame = self.camera_frame_id
        laser_frame = msg.header.frame_id
        
        injected_count = 0
        
        try:
            trans = self.tf_buffer.lookup_transform(
                laser_frame,
                camera_frame,
                rclpy.time.Time())
                
            for pt in self.latest_3d_points:
                # Create PointStamped
                p = PointStamped()
                p.header.frame_id = camera_frame
                # Ensure native Python float types (ROS2 rejects numpy types)
                p.point.x = float(pt[0])
                p.point.y = float(pt[1])
                p.point.z = float(pt[2])
                
                # Transform
                p_laser = do_transform_point(p, trans)
                
                lx = p_laser.point.x
                ly = p_laser.point.y
                
                # Convert to Polar
                dist = math.sqrt(lx*lx + ly*ly)
                angle = math.atan2(ly, lx)
                
                # Check bounds and filter noise near robot (e.g., ground reflections under vehicle)
                if angle < fused_scan.angle_min or angle > fused_scan.angle_max:
                    continue
                
                # Validate the transformed point is reasonable
                if not (-10.0 < lx < 10.0 and -10.0 < ly < 10.0):
                    continue
                    
                # Ignore points too close to the robot center (footprint exclusion)
                if dist < 0.15: 
                    continue
                    
                if dist > fused_scan.range_max:
                    continue
                    
                # Find index
                idx = int((angle - fused_scan.angle_min) / fused_scan.angle_increment)
                
                # Inject obstacle if closer
                if 0 <= idx < len(ranges):
                    current_r = ranges[idx]
                    # If current_r is inf or 0 (unknown/far) or further away
                    if math.isinf(current_r) or math.isnan(current_r) or current_r == 0.0 or dist < current_r:
                        ranges[idx] = dist
                        injected_count += 1
                        
            fused_scan.ranges = ranges
            self.scan_pub.publish(fused_scan)
            
            if injected_count > 0:
                self.get_logger().info(f'Injected {injected_count} lane points into scan (frame: {laser_frame})', 
                                       throttle_duration_sec=2.0)

        except TransformException as ex:
            # If TF fails, just republish original
            self.get_logger().warn(f'Could not transform lane points from {camera_frame} to {laser_frame}: {ex}', 
                                   throttle_duration_sec=5.0)
            self.scan_pub.publish(msg)

    def depth_callback(self, msg):
        """Store the latest depth image"""
        try:
            # Depth image is usually 16UC1 (uint16 in mm) or 32FC1 (float in meters)
            # The libgazebo_ros_camera plugin usually sends 32FC1
            self.latest_depth_image = image_msg_to_cv(msg)
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

        # HSV for cone rejection (see verify_line_profile) - grayscale alone
        # can't tell a cone's white stripe apart from real lane paint.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) if self.reject_orange_cones else None
        
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
        valid_lines = self.filter_valid_lanes(lines, gray, height, width, hsv) if lines is not None else []
        
        return valid_lines, white_mask, edges
    
    def filter_valid_lanes(self, lines, gray, height, width, hsv=None):
        """
        Filter for lines that look like lane markings (white on dark)
        """
        valid_lines = []
        if lines is None:
            return []

        # OpenCV 4 returns (N, 1, 4) and OpenCV 5 (N, 4); flatten to one line per row.
        for line in np.asarray(lines).reshape(-1, 4):
            x1, y1, x2, y2 = line

            # Calculate line length
            length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
            if length < self.min_line_length:
                continue

            # Calculate angle (reject very horizontal lines, but be lenient)
            angle = np.abs(np.arctan2(y2-y1, x2-x1))
            if angle < 0.05: # Too horizontal
                continue

            # Verify Profile: Dark - White - Dark
            if self.verify_line_profile(gray, x1, y1, x2, y2, hsv):
                valid_lines.append(line)

        return valid_lines

    def verify_line_profile(self, gray, x1, y1, x2, y2, hsv=None):
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
        if not (is_dark_1 or is_dark_2):
            return False

        # Reject cone/barrel stripes: their white band is genuinely white and
        # often has dark ground on one side too, so the checks above alone
        # can't tell it apart from paint. What it always has that paint never
        # does is saturated orange material bordering the stripe. A fixed
        # small patch centered on the line isn't enough to find it though -
        # on a wide object (a barrel's band can be tens of pixels thick) that
        # patch can sit entirely inside the white band, never reaching the
        # orange border. Instead, walk outward along the perpendicular on
        # both sides until either the orange is found or the search gives up.
        if hsv is not None and self.has_nearby_orange(hsv, mx, my, nx, ny):
            return False

        return True

    def has_nearby_orange(self, hsv, x, y, nx, ny, near=None, far=None, steps=10):
        """
        True if a saturated-orange pixel is found by walking outward along
        (nx, ny) - the line's perpendicular - from `near` to `far` pixels,
        on both sides.
        """
        if near is None:
            near = self.orange_check_radius
        if far is None:
            far = self.orange_search_max_radius

        h, w = hsv.shape[:2]
        for sign in (1.0, -1.0):
            for t in np.linspace(near, far, steps):
                px, py = int(x + sign * nx * t), int(y + sign * ny * t)
                if 0 <= py < h and 0 <= px < w:
                    hh, ss, vv = hsv[py, px]
                    if (self.orange_hue_low <= hh <= self.orange_hue_high
                            and ss >= self.orange_sat_min
                            and vv >= self.orange_val_min):
                        return True
        return False

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
            cv_image = image_msg_to_cv(msg, desired_encoding='bgr8')
            self.img_h, self.img_w = cv_image.shape[:2]
            self.update_ground_tf()
            
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
                
                cv2.imshow(f'Lane Detection - {self.get_name()}', combined_resized)
                cv2.waitKey(1)
            
            # Publish markers for detected lanes
            if lines and len(self.latest_lines) > 0:
                # Convert to format expected by publish_lane_markers
                lines_array = [[line] for line in self.latest_lines]
                header = msg.header
                header.frame_id = self.camera_frame_id # Use this instance's own optical frame for projection
                
                self.publish_lane_markers(lines_array, header)
                
                # Also publish to PointCloud for Nav2 costmap
                if self.latest_depth_image is not None or self.ground_tf is not None:
                    self.publish_obstacle_cloud(self.latest_lines, self.latest_depth_image, header)
                else:
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
        
        fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
        
        # Camera height and tilt for ground plane fallback
        camera_height = self.camera_mount_height  # meters above ground
        camera_pitch = self.camera_mount_pitch  # radians (0 = looking straight ahead)
        
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
                    
                    if not (0 <= u < self.img_w and 0 <= v < self.img_h):
                        continue
                        
                    d = depth_img[v, u] if depth_img is not None else float('nan')
                    
                    # Convert to native Python float to handle numpy types
                    d = float(d)
                    
                    # Check if depth is valid
                    use_fallback = False
                    if math.isnan(d) or math.isinf(d) or d <= 0.05 or d > 10.0:
                        use_fallback = True
                    
                    # If depth is invalid, estimate using ground plane assumption
                    if use_fallback and self.ground_tf is not None:
                        p = self.ground_point(u, v)
                        if p is None:
                            continue
                        x, y, z = p
                        fallback_count += 1
                    elif use_fallback:
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

                    if z > self.max_lane_range:
                        continue

                    # Add to points list (as native Python floats)
                    points.append([float(x), float(y), float(z)])
                    self.latest_3d_points.append((float(x), float(y), float(z)))
        
        self.get_logger().info(f'Generated {len(points)} 3D points from {len(lines)} lane segments '\
                               f'(valid_depth={valid_depth_count}, fallback={fallback_count})', 
                               throttle_duration_sec=2.0)
        
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

        fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
        camera_height = self.camera_mount_height
        camera_pitch = self.camera_mount_pitch

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

                if not (0 <= u < self.img_w and 0 <= v < self.img_h):
                    continue

                x_3d, y_3d, z_3d = None, None, None

                if self.latest_depth_image is not None:
                    d = self.latest_depth_image[v, u]
                    if not np.isnan(d) and not np.isinf(d) and 0.1 < d < 10.0:
                        z_3d = float(d)
                        x_3d = float((u - cx) * z_3d / fx)
                        y_3d = float((v - cy) * z_3d / fy)

                if x_3d is None and self.ground_tf is not None:
                    p = self.ground_point(u, v)
                    if p is not None:
                        x_3d, y_3d, z_3d = p
                elif x_3d is None:
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

                if z_3d > self.max_lane_range:
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