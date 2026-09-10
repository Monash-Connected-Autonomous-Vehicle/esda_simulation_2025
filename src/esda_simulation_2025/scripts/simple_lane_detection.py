#!/usr/bin/env python3
"""
Simple lane detection: every white pixel is an obstacle.

No line fitting, left/right classification or virtual lanes. White pixels
below the horizon are projected to 3D (depth image, or the ground plane where
depth is invalid) and published as obstacles:

  /lane_obstacles  PointCloud2 for the costmaps
  /scan_fused      via the base class scan_callback (motion-compensated),
                   which also puts the white into the SLAM map

No /lane_markers are published, so the waypoint navigator's lane-following
goals stay idle and it avoids white like any other obstacle.
"""

import math

import cv2
import numpy as np
import rclpy

from lane_detection import LaneDetectionNode


# ==========================================================================
# Pure helper - no ROS state, so tools/validate_waypoint_logic.py can test it
# offline.
# ==========================================================================

def project_pixels_to_camera(us, vs, depth, fx, fy, cx, cy,
                             camera_height, camera_pitch, max_range):
    """
    Project image pixels to 3D points in camera_link_optical (x right, y down,
    z forward), vectorised.

    Same per-pixel rules as LaneDetectionNode.publish_obstacle_cloud:
      - depth is used when finite and in (0.05, 10] m; the point is kept if
        0.2 <= z <= 8 and |x| <= 5
      - otherwise the pixel ray is intersected with the ground plane, kept if
        the ray points down (|ray_y| > 0.01), 0.3 < t < 8, z > 0.2, |x| < 5
    plus a max_range cap on z, so far (inaccurate) white is dropped.

    Returns an N x 3 float64 array, in input pixel order.
    """
    us = np.asarray(us, dtype=np.float64)
    vs = np.asarray(vs, dtype=np.float64)

    if depth is not None:
        d = depth[vs.astype(np.int64), us.astype(np.int64)].astype(np.float64)
    else:
        d = np.full(us.shape, np.nan)

    valid_depth = np.isfinite(d) & (d > 0.05) & (d <= 10.0)

    # Depth path
    z_d = d
    x_d = (us - cx) * z_d / fx
    y_d = (vs - cy) * z_d / fy
    keep_d = valid_depth & (z_d >= 0.2) & (z_d <= 8.0) & (np.abs(x_d) <= 5.0)

    # Ground-plane fallback
    ray_x = (us - cx) / fx
    ray_y = (vs - cy) / fy
    ray_z = np.ones_like(ray_x)
    ray_len = np.sqrt(ray_x ** 2 + ray_y ** 2 + ray_z ** 2)
    ray_x = ray_x / ray_len
    ray_y = ray_y / ray_len
    ray_z = ray_z / ray_len

    ray_y_rot = ray_y * math.cos(camera_pitch) - ray_z * math.sin(camera_pitch)
    ray_z_rot = ray_y * math.sin(camera_pitch) + ray_z * math.cos(camera_pitch)

    with np.errstate(divide='ignore', invalid='ignore'):
        t = camera_height / ray_y_rot

    z_g = t * ray_z_rot
    x_g = t * ray_x
    y_g = t * ray_y_rot
    keep_g = (
        ~valid_depth
        & (np.abs(ray_y_rot) > 0.01) & (t > 0.3) & (t < 8.0)
        & (z_g > 0.2) & (np.abs(x_g) < 5.0)
    )

    x = np.where(valid_depth, x_d, x_g)
    y = np.where(valid_depth, y_d, y_g)
    z = np.where(valid_depth, z_d, z_g)

    keep = (keep_d | keep_g) & (z <= max_range)

    return np.column_stack([x[keep], y[keep], z[keep]])


class SimpleLaneDetectionNode(LaneDetectionNode):
    """
    White-pixel obstacle detector. Reuses the base class's subscriptions,
    depth caching, TF, store_lane_points / publish_point_cloud and the
    motion-compensated /scan_fused injection; only image_callback differs.
    """

    def __init__(self):
        super().__init__()

        # White = bright and unsaturated (HSV). Orange barrel paint is
        # saturated so it is excluded; its white stripes are obstacles anyway.
        self.declare_parameter('white_value_min', 200)
        self.declare_parameter('white_saturation_max', 40)
        # Ignore rows above this fraction of the image height (the camera
        # looks straight ahead, so the horizon sits at mid-image).
        self.declare_parameter('roi_top_fraction', 0.5)
        # Sample every Nth white pixel in each direction.
        self.declare_parameter('pixel_stride', 4)
        # Drop white further than this (m). Matches the costmaps'
        # obstacle_max_range, and keeps inaccurate far white out of the map.
        self.declare_parameter('max_range', 5.0)

        self.white_value_min = self.get_parameter('white_value_min').value
        self.white_saturation_max = self.get_parameter('white_saturation_max').value
        self.roi_top_fraction = self.get_parameter('roi_top_fraction').value
        self.pixel_stride = max(1, int(self.get_parameter('pixel_stride').value))
        self.max_range = self.get_parameter('max_range').value

        self.get_logger().info(
            'Simple lane detection: white pixels -> obstacles '
            '(/lane_obstacles, /scan_fused); no /lane_markers')

    def white_mask(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = (
            (hsv[:, :, 2] >= self.white_value_min)
            & (hsv[:, :, 1] <= self.white_saturation_max)
        ).astype(np.uint8) * 255

        mask[:int(mask.shape[0] * self.roi_top_fraction), :] = 0

        # Remove isolated speckle while keeping thin painted lines.
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        mask = self.white_mask(image)

        stride = self.pixel_stride
        vs, us = np.nonzero(mask[::stride, ::stride])
        vs = vs * stride
        us = us * stride

        depth = self.latest_depth_image
        if depth is not None and depth.shape[:2] != mask.shape:
            self.get_logger().warn(
                f'Depth {depth.shape[:2]} and image {mask.shape} sizes differ; '
                f'using ground-plane projection only',
                throttle_duration_sec=5.0)
            depth = None

        points = project_pixels_to_camera(
            us, vs, depth, self.fx, self.fy, self.cx, self.cy,
            self.camera_height, self.camera_pitch, self.max_range)

        header = msg.header
        header.frame_id = 'camera_link_optical'

        if len(points) > 0:
            self.store_lane_points(points, header)
            self.publish_point_cloud(points, header)
        else:
            # Nothing white this frame: stop injecting the previous detection.
            self.latest_3d_points = []

        self.get_logger().info(
            f'{len(points)} white obstacle points', throttle_duration_sec=2.0)

        if self.show_viz:
            overlay = image.copy()
            overlay[mask > 0] = (0, 0, 255)
            viz = cv2.addWeighted(image, 0.5, overlay, 0.5, 0)
            cv2.putText(viz, f'WHITE OBSTACLE POINTS: {len(points)}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Simple Lane Detection', viz)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = SimpleLaneDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
