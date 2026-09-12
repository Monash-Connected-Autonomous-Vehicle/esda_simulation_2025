#!/usr/bin/env python3
"""
FCN Lane Detection Node for ESDA Simulation
Keeps the original lane_detection.py untouched by subclassing it and
overriding only the lane mask/line extraction stage.

This implementation is adapted from:
lane-detection-on-rural-roads-master/CS542_Project/Code/draw.py
to keep behavior close to the original repo inference pipeline.
"""

import os
import importlib
from collections import deque

import cv2
import numpy as np
import rclpy

from lane_detection import LaneDetectionNode


class LaneDetectionFCNNode(LaneDetectionNode):
    def __init__(self):
        super().__init__()

        # self.declare_parameter('show_visualization', True)
        # Keep default path aligned with the cloned repo layout in this workspace
        self.declare_parameter(
            'fcn_model_path',
            'lane-detection-on-rural-roads-master/CS542_Project/Code/FCN_model.h5'
        )
        self.declare_parameter('fcn_input_width', 160)
        self.declare_parameter('fcn_input_height', 80)
        # Original repo does not normalize input to [0,1], so default scale = 1.0
        self.declare_parameter('fcn_input_scale', 1.0)
        # Original repo uses prediction*255 + visualization; threshold on that space
        self.declare_parameter('fcn_mask_threshold', 140.0)
        self.declare_parameter('fcn_temporal_window', 5)
        self.declare_parameter('fcn_use_roi', True)
        self.declare_parameter('fcn_draw_area', True)
        self.declare_parameter('fcn_overlay_alpha', 1.0)

        self.fcn_model_path = self.get_parameter('fcn_model_path').value
        self.fcn_input_width = int(self.get_parameter('fcn_input_width').value)
        self.fcn_input_height = int(self.get_parameter('fcn_input_height').value)
        self.fcn_input_scale = float(self.get_parameter('fcn_input_scale').value)
        self.fcn_mask_threshold = float(self.get_parameter('fcn_mask_threshold').value)
        self.fcn_temporal_window = int(self.get_parameter('fcn_temporal_window').value)
        self.fcn_use_roi = bool(self.get_parameter('fcn_use_roi').value)
        self.fcn_draw_area = bool(self.get_parameter('fcn_draw_area').value)
        self.fcn_overlay_alpha = float(self.get_parameter('fcn_overlay_alpha').value)
        # self.show_viz = self.get_parameter('show_visualization').value


        self.tf_module = None
        self.fcn_model = None
        self.fcn_model_ready = False
        self.model_resolved_path = None

        self.recent_predictions = deque(maxlen=max(self.fcn_temporal_window, 1))

        self._initialize_fcn_model()

        self.get_logger().info('FCN lane detection override active (original lane_detection.py unchanged)')

    def _resolve_model_path(self, raw_path: str):
        if not raw_path:
            return None

        expanded = os.path.expanduser(raw_path)
        if os.path.isabs(expanded) and os.path.exists(expanded):
            return expanded

        # Candidate roots to support running from source or install tree
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.join(os.getcwd(), expanded),
            os.path.join(script_dir, expanded),
            os.path.join(script_dir, '..', '..', '..', '..', expanded),
            os.path.join(script_dir, '..', '..', '..', '..', '..', '..', expanded),
        ]

        for candidate in candidates:
            candidate_abs = os.path.abspath(candidate)
            if os.path.exists(candidate_abs):
                return candidate_abs

        return None

    def _initialize_fcn_model(self):
        resolved_path = self._resolve_model_path(self.fcn_model_path)
        if resolved_path is None:
            self.get_logger().warn(
                f'FCN model not found from fcn_model_path="{self.fcn_model_path}". '
                'Falling back to classic detector.'
            )
            return

        try:
            self.tf_module = importlib.import_module('tensorflow')
        except Exception as exc:
            self.get_logger().warn(f'TensorFlow not available ({exc}). Falling back to classic detector.')
            return

        try:
            self.fcn_model = self.tf_module.keras.models.load_model(resolved_path)
            self.fcn_model_ready = True
            self.model_resolved_path = resolved_path
            self.get_logger().info(
                f'Loaded FCN model: {resolved_path} '
                f'(input={self.fcn_input_width}x{self.fcn_input_height}, '
                f'scale={self.fcn_input_scale}, threshold={self.fcn_mask_threshold:.1f}, '
                f'temporal_window={self.fcn_temporal_window})'
            )
        except Exception as exc:
            self.fcn_model_ready = False
            self.get_logger().error(f'Failed to load FCN model: {exc}. Falling back to classic detector.')

# def draw_drivable_area_overlay(self, image, lines):
    
    def draw_drivable_area_overlay(self,image,lines):
        if lines is None or len(lines) < 2:
            return image

        h, w = image.shape[:2]

        left_lines = []
        right_lines = []

        for line in lines:
            x1, y1, x2, y2 = line

            if x2 == x1:
                continue

            slope = (y2 - y1) / (x2 - x1)

            # ignore flat lines
            if abs(slope) < 0.3:
                continue

            if slope < 0:
                left_lines.append(line)
            else:
                right_lines.append(line)

        if len(left_lines) == 0 or len(right_lines) == 0:
            return image

        def average_line(line_list):
            x_bottoms = []
            x_tops = []

            y_bottom = h
            y_top = int(0.60 * h)

            for x1, y1, x2, y2 in line_list:
                if x2 == x1:
                    continue

                m = (y2 - y1) / (x2 - x1)
                b = y1 - m * x1

                x_bottom = int((y_bottom - b) / m)
                x_top = int((y_top - b) / m)

                x_bottoms.append(x_bottom)
                x_tops.append(x_top)

            if len(x_bottoms) == 0:
                return None

            return (
                int(np.mean(x_bottoms)), y_bottom,
                int(np.mean(x_tops)), y_top
            )

        left_line = average_line(left_lines)
        right_line = average_line(right_lines)

        if left_line is None or right_line is None:
            return image

        lx1, ly1, lx2, ly2 = left_line
        rx1, ry1, rx2, ry2 = right_line

        polygon = np.array([[
            (lx1, ly1),
            (rx1, ry1),
            (rx2, ry2),
            (lx2, ly2)
        ]], dtype=np.int32)

        overlay = image.copy()
        cv2.fillPoly(overlay, polygon, (255, 0, 0))

        alpha = 0.45
        return cv2.addWeighted(overlay, alpha, image, 1.0 - alpha, 0.0)

    def detect_white_lanes(self, image):
        # return super().detect_white_lanes(image)
        if not self.fcn_model_ready or self.fcn_model is None:
            return super().detect_white_lanes(image)

        input_w = self.fcn_input_width
        input_h = self.fcn_input_height

        resized = cv2.resize(image, (input_w, input_h), interpolation=cv2.INTER_AREA)
        resized_rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        model_input = resized_rgb.astype(np.float32) * self.fcn_input_scale
        model_input = np.expand_dims(model_input, axis=0)

        prediction = self.fcn_model.predict(model_input, verbose=0)
        lane_pred = prediction[0]

        if lane_pred.ndim == 3 and lane_pred.shape[-1] == 1:
            lane_pred = lane_pred[:, :, 0]

        lane_pred = lane_pred.astype(np.float32) * 255.0
        self.recent_predictions.append(lane_pred)
        lane_avg = np.mean(np.stack(list(self.recent_predictions), axis=0), axis=0)
        lane_avg = np.clip(lane_avg, 0.0, 255.0)

        small_mask = (lane_avg >= self.fcn_mask_threshold).astype(np.uint8) * 255

        if self.fcn_use_roi:
            roi = np.zeros_like(small_mask)
            h, w = small_mask.shape

            roi_polygon = np.array([[
                (int(0.10 * w), h),
                (int(0.90 * w), h),
                (int(0.62 * w), int(0.72 * h)),
                (int(0.38 * w), int(0.72 * h)),
            ]], dtype=np.int32)

            cv2.fillPoly(roi, roi_polygon, 255)
            small_mask = cv2.bitwise_and(small_mask, roi)

        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 3))
        horizontal_blobs = cv2.morphologyEx(small_mask, cv2.MORPH_OPEN, horizontal_kernel)
        small_mask = cv2.subtract(small_mask, horizontal_blobs)

        kernel = np.ones((3, 3), np.uint8)
        small_mask = cv2.morphologyEx(small_mask, cv2.MORPH_OPEN, kernel)
        small_mask = cv2.morphologyEx(small_mask, cv2.MORPH_CLOSE, kernel)

        white_mask = cv2.resize(
            small_mask,
            (image.shape[1], image.shape[0]),
            interpolation=cv2.INTER_NEAREST
        )

        edges = cv2.Canny(white_mask, 40, 120)

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=12,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )

        filtered_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                dx = x2 - x1
                dy = y2 - y1

                if dx == 0:
                    angle = 90.0
                else:
                    angle = abs(np.degrees(np.arctan2(dy, dx)))

                # Reject mostly-horizontal map/wall edges
                if angle < 12.0:
                    continue

                filtered_lines.append([x1, y1, x2, y2])
        else:
            filtered_lines = []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        valid_lines = self.filter_valid_lanes(
            np.array([[line] for line in filtered_lines], dtype=np.int32),
            gray, image.shape[0], image.shape[1]
        ) if len(filtered_lines) > 0 else []

        return valid_lines, white_mask, edges

    def draw_lane_area_overlay(self, image, white_mask):
        """
        Draw filled lane area in blue similar to the original repo draw.py style.
        """
        if white_mask is None:
            return image

        soft_mask = cv2.GaussianBlur(white_mask, (5, 5), 0)
        overlay = np.zeros_like(image)
        overlay[:, :, 0] = soft_mask  # B channel in BGR

        alpha = max(0.0, min(self.fcn_overlay_alpha, 1.5))
        return cv2.addWeighted(image, 1.0, overlay, alpha, 0.0)

    def image_callback(self, msg):
        """
        Process incoming camera images using FCN lane extraction while keeping
        the rest of the classic lane_detection.py pipeline intact.
        """
        self.get_logger().info('Image callback received', throttle_duration_sec=5.0)
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect lanes using FCN mask + classic geometric filtering
            lines, white_mask, edges = self.detect_white_lanes(cv_image)

            # Create visualization image
            viz_image = cv_image.copy()
            detection_image = np.zeros_like(cv_image)
            self.latest_lines = []

            if lines and len(lines) > 0:
                for line in lines:
                    x1, y1, x2, y2 = line
                    self.latest_lines.append(line)

                    # Draw standard line overlay for debugging
                    cv2.line(detection_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    if not self.fcn_draw_area:
                        cv2.line(viz_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

                self.get_logger().info(f'Detected {len(lines)} lane segments', throttle_duration_sec=2.0)

            if self.show_viz:
                white_mask_colored = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)
                edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

                if self.fcn_draw_area:
                    left_view = self.draw_drivable_area_overlay(viz_image, self.latest_lines)
                else:
                    left_view = viz_image.copy()

                right_view = self.latest_right_image.copy() if self.latest_right_image is not None else np.zeros_like(cv_image)

                stereo_row = np.hstack([left_view, right_view])
                processing_row = np.hstack([white_mask_colored, edges_colored if edges_colored is not None else detection_image])
                combined = np.vstack([stereo_row, processing_row])
                combined_resized = cv2.resize(combined, (1280, 720))

                cv2.putText(combined_resized, 'LIVE STEREO: LEFT', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(combined_resized, 'LIVE STEREO: RIGHT', (650, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(combined_resized, 'FCN MASK', (10, 390),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(combined_resized, f'DETECTED SEGMENTS: {len(lines) if lines is not None else 0}',
                            (650, 390),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(combined_resized, 'VIEW: AREA OVERLAY' if self.fcn_draw_area else 'VIEW: LINE OVERLAY',
                            (650, 430),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

                cv2.imshow('Lane Detection - ESDA STEREO (FCN)', combined_resized)
                cv2.waitKey(1)

            if lines and len(self.latest_lines) > 0:
                # Convert to format expected by publish_lane_markers
                lines_array = [[line] for line in self.latest_lines]
                header = msg.header
                header.frame_id = 'camera_link_optical'  # Use optical frame for projection

                self.publish_lane_markers(lines_array, header)

                if self.latest_depth_image is not None:
                    self.publish_obstacle_cloud(self.latest_lines, self.latest_depth_image, header)
                else:
                    self.latest_3d_points = []
            else:
                self.latest_3d_points = []

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionFCNNode()

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
