#!/usr/bin/env python3
"""
TwinLiteNet+ Lane Detection Node for ESDA Simulation
Keeps the original lane_detection.py untouched by subclassing it and
overriding only the lane mask/line extraction stage, mirroring the
lane_detection_FCN.py pattern so all three detectors (Regular/FCN/TwinLite)
share identical 3D projection, marker publishing, and /scan_fused injection.

Model: TwinLiteNetPlus (https://github.com/chequanghuy/TwinLiteNetPlus)
Requires the upstream repo cloned locally -- the .pth checkpoint alone is
not enough, PyTorch needs the matching nn.Module class definitions to load
a state_dict. See LANE_DETECTION.md for setup steps.
"""

import os
import sys
import time
import argparse
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import (qos_profile_sensor_data, qos_profile_system_default)
from sensor_msgs.msg import Image, CameraInfo

from lane_detection import LaneDetectionNode



def _letterbox_for_img(img, new_shape=640, color=(114, 114, 114), auto=True):
    """
    Ported verbatim from TwinLiteNetPlus/demoDataset.py::letterbox_for_img.
    Preprocessing must match this exactly -- the pretrained checkpoints were
    trained on images resized this way (minimum-rectangle padding to the
    nearest 32px, not a forced square), and diverging here produces silently
    wrong predictions rather than an error.
    """
    shape = img.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]

    if auto:
        dw, dh = np.mod(dw, 32), np.mod(dh, 32)

    dw /= 2
    dh /= 2
    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_AREA)

    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, (dw, dh)


class LaneDetectionTwinLiteNode(LaneDetectionNode):
    def __init__(self):
        super().__init__()

        # self.declare_parameter('show_visualization', True)
        # Keep default paths aligned with the cloned repo layout in this workspace
        self.declare_parameter('twinlite_repo_path', 'TwinLiteNetPlus')
        self.declare_parameter('twinlite_weight_path', 'TwinLiteNetPlus/pretrained/nano.pth')
        self.declare_parameter('twinlite_variant', 'nano')  # nano/small/medium/large
        self.declare_parameter('twinlite_device', 'auto')   # auto/cpu/cuda
        self.declare_parameter('twinlite_img_size', 640)
        self.declare_parameter('twinlite_lane_confidence', 0.5)
        self.declare_parameter('twinlite_temporal_window', 5)
        self.declare_parameter('twinlite_draw_area', True)
        self.declare_parameter('twinlite_overlay_alpha', 0.45)
        # 0 = uncapped; throttle exists because CPU inference for this model
        # is slow enough to matter (see LANE_DETECTION.md perf note) and a
        # RELIABLE/KEEP_LAST(10) image topic will otherwise buffer stale
        # frames behind a slow callback.
        self.declare_parameter('twinlite_max_inference_hz', 0.0)

        self.twinlite_repo_path = self.get_parameter('twinlite_repo_path').value
        self.twinlite_weight_path = self.get_parameter('twinlite_weight_path').value
        self.twinlite_variant = self.get_parameter('twinlite_variant').value
        self.twinlite_device_pref = self.get_parameter('twinlite_device').value
        self.twinlite_img_size = int(self.get_parameter('twinlite_img_size').value)
        self.twinlite_lane_confidence = float(self.get_parameter('twinlite_lane_confidence').value)
        self.twinlite_temporal_window = int(self.get_parameter('twinlite_temporal_window').value)
        self.twinlite_draw_area = bool(self.get_parameter('twinlite_draw_area').value)
        self.twinlite_overlay_alpha = float(self.get_parameter('twinlite_overlay_alpha').value)
        self.twinlite_max_inference_hz = float(self.get_parameter('twinlite_max_inference_hz').value)
        # self.show_viz = self.get_parameter('show_visualization').value


        self.torch = None
        self.twinlite_model = None
        self.twinlite_model_ready = False
        self.twinlite_device = None
        self._latest_da_mask = None
        self._last_infer_time = 0.0

        self.recent_lane_probs = deque(maxlen=max(self.twinlite_temporal_window, 1))

        self.last_valid_lines = []
        self.last_valid_line_time = self.get_clock().now()

        # Continue publishing the previous valid lanes for 0.75 seconds
        # when one TwinLite/Hough frame temporarily fails.
        self.lane_hold_time = 0.75
        self.last_valid_line_time = self.get_clock().now()

        # Route the image subscription to its own reentrant callback group so a
        # slow forward pass can never block scan_callback's TF lookup and
        # /scan_fused republish -- that path feeds the Nav2 costmap, so stalling
        # it is a navigation-latency issue, not just a viz-lag one.
        self._reroute_image_subscription()

        self._initialize_twinlite_model()

        self.drivable_mask_pub = self.create_publisher(
            Image,
            '/lane/drivable_area_mask',
            qos_profile_sensor_data

        )

        self.drivable_overlay_pub = self.create_publisher(
            Image,
            '/lane/drivable_area_overlay',
            qos_profile_sensor_data
        )

        self.get_logger().info('TwinLiteNet+ lane detection override active (original lane_detection.py unchanged)')

    def _reroute_image_subscription(self):
        self.inference_callback_group = MutuallyExclusiveCallbackGroup()
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.destroy_subscription(self.image_sub)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/left/image_raw',
            self.image_callback,
            image_qos,
            callback_group=self.inference_callback_group
        )

    def _resolve_path(self, raw_path: str):
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

    def _initialize_twinlite_model(self):
        resolved_repo = self._resolve_path(self.twinlite_repo_path)
        if resolved_repo is None or not os.path.isdir(resolved_repo):
            self.get_logger().warn(
                f'TwinLiteNetPlus repo not found from twinlite_repo_path="{self.twinlite_repo_path}". '
                'Falling back to classic detector.'
            )
            return

        resolved_weight = self._resolve_path(self.twinlite_weight_path)
        if resolved_weight is None:
            self.get_logger().warn(
                f'TwinLiteNetPlus weight file not found from twinlite_weight_path="{self.twinlite_weight_path}". '
                'Falling back to classic detector.'
            )
            return

        try:
            import torch
            self.torch = torch
        except Exception as exc:
            self.get_logger().warn(f'PyTorch not available ({exc}). Falling back to classic detector.')
            return

        if resolved_repo not in sys.path:
            sys.path.insert(0, resolved_repo)

        try:
            from model.model import TwinLiteNetPlus  # provided by the cloned repo, not this package
        except Exception as exc:
            self.get_logger().error(
                f'Failed to import TwinLiteNetPlus model definition from {resolved_repo}: {exc}. '
                'Falling back to classic detector.'
            )
            return

        device_pref = self.twinlite_device_pref
        if device_pref == 'auto':
            device_str = 'cuda' if self.torch.cuda.is_available() else 'cpu'
        elif device_pref == 'cuda' and not self.torch.cuda.is_available():
            self.get_logger().warn('twinlite_device="cuda" requested but CUDA is not available on this torch build; using CPU.')
            device_str = 'cpu'
        else:
            device_str = device_pref

        try:
            model_args = argparse.Namespace(config=self.twinlite_variant)
            model = TwinLiteNetPlus(model_args)

            state_dict = self.torch.load(resolved_weight, map_location=device_str)
            # Defensive unwrap: some checkpoints save {'state_dict': ...} or
            # add a 'module.' prefix from DataParallel training. Try the raw
            # dict first, only unwrap if keys don't match.
            if isinstance(state_dict, dict) and 'state_dict' in state_dict and not any(
                k in state_dict for k in model.state_dict().keys()
            ):
                state_dict = state_dict['state_dict']
            if isinstance(state_dict, dict) and any(k.startswith('module.') for k in state_dict.keys()):
                state_dict = {k.replace('module.', '', 1): v for k, v in state_dict.items()}

            model.load_state_dict(state_dict)
            model.to(device_str)
            model.eval()

            self.twinlite_model = model
            self.twinlite_device = device_str
            self.twinlite_model_ready = True
            self.get_logger().info(
                f'Loaded TwinLiteNetPlus[{self.twinlite_variant}]: {resolved_weight} '
                f'on device={device_str}, img_size={self.twinlite_img_size}, '
                f'lane_confidence={self.twinlite_lane_confidence:.2f}, '
                f'temporal_window={self.twinlite_temporal_window}'
            )
            if device_str == 'cpu':
                self.get_logger().warn(
                    'TwinLiteNetPlus running on CPU -- expect noticeably higher latency than the '
                    'classic/FCN detectors. Use twinlite_variant:=nano and/or twinlite_max_inference_hz '
                    'to bound it if /scan_fused starts lagging.'
                )
        except Exception as exc:
            self.twinlite_model_ready = False
            self.get_logger().error(f'Failed to load TwinLiteNetPlus model: {exc}. Falling back to classic detector.')

    def detect_white_lanes(self, image):
        if not self.twinlite_model_ready or self.twinlite_model is None:
            return super().detect_white_lanes(image)

        torch = self.torch
        h_img, w_img = image.shape[:2]

        letterboxed, (pad_w, pad_h) = _letterbox_for_img(
            image,
            new_shape=self.twinlite_img_size,
            auto=True
        )

        rgb = letterboxed[:, :, ::-1]
        chw = np.ascontiguousarray(rgb.transpose(2, 0, 1))

        tensor = (
            torch.from_numpy(chw)
            .float()
            .div(255.0)
            .unsqueeze(0)
            .to(self.twinlite_device)
        )

        padded_h = tensor.shape[2]
        padded_w = tensor.shape[3]

        pad_w_i = int(pad_w)
        pad_h_i = int(pad_h)

        y_end = padded_h - pad_h_i if pad_h_i > 0 else padded_h
        x_end = padded_w - pad_w_i if pad_w_i > 0 else padded_w

        with torch.no_grad():
            da_logits, ll_logits = self.twinlite_model(tensor)

            ll_crop = ll_logits[
                :,
                :,
                pad_h_i:y_end,
                pad_w_i:x_end
            ]

            ll_up = torch.nn.functional.interpolate(
                ll_crop,
                size=(h_img, w_img),
                mode='bilinear',
                align_corners=False
            )

            lane_prob = (
                torch.softmax(ll_up, dim=1)[0, 1]
                .detach()
                .cpu()
                .numpy()
            )

            if self.twinlite_draw_area:
                da_crop = da_logits[
                    :,
                    :,
                    pad_h_i:y_end,
                    pad_w_i:x_end
                ]

                da_up = torch.nn.functional.interpolate(
                    da_crop,
                    size=(h_img, w_img),
                    mode='bilinear',
                    align_corners=False
                )

                self._latest_da_mask = (
                    torch.argmax(da_up, dim=1)[0]
                    .detach()
                    .cpu()
                    .numpy() > 0
                ).astype(np.uint8) * 255

                self._latest_da_mask[-20:, :] = 0

            else:
                self._latest_da_mask = None

        # Temporal smoothing
        self.recent_lane_probs.append(lane_prob)

        lane_prob_avg = np.mean(
            np.stack(list(self.recent_lane_probs), axis=0),
            axis=0
        )

        lane_mask = (
            lane_prob_avg >= self.twinlite_lane_confidence
        ).astype(np.uint8) * 255

        # Clean the segmentation mask
        kernel = np.ones((3, 3), np.uint8)

        lane_mask = cv2.morphologyEx(
            lane_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        lane_mask = cv2.morphologyEx(
            lane_mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # Ignore upper image region
        roi_mask = lane_mask.copy()
        roi_top = int(0.30 * h_img)
        roi_mask[:roi_top, :] = 0

        edges = cv2.Canny(
            roi_mask,
            30,
            100
        )

        hough_lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=8,
            minLineLength=8,
            maxLineGap=30
        )

        twinlite_lines = []

        if hough_lines is not None:
            for detected_line in hough_lines:
                x1, y1, x2, y2 = detected_line[0]

                dx = x2 - x1
                dy = y2 - y1

                angle = (
                    90.0
                    if dx == 0
                    else abs(np.degrees(np.arctan2(dy, dx)))
                )

                # Only reject lines that are almost horizontal
                if angle < 5.0:
                    continue

                twinlite_lines.append([
                    int(x1),
                    int(y1),
                    int(x2),
                    int(y2)
                ])

        

        self.get_logger().info(
            f'TwinLite mask pixels={cv2.countNonZero(lane_mask)}, '
            f'Hough lines={len(twinlite_lines)}',
            throttle_duration_sec=2.0
        )

        # Important: no classical filter_valid_lanes() here.
        return twinlite_lines, lane_mask, edges

    def draw_drivable_area_overlay(self, image, da_mask):
        if da_mask is None:
            return image
        overlay = np.zeros_like(image)
        overlay[:, :, 0] = da_mask  # B channel in BGR, mirrors FCN's blue drivable-area tint
        alpha = max(0.0, min(self.twinlite_overlay_alpha, 1.0))
        return cv2.addWeighted(image, 1.0, overlay, alpha, 0.0)

    def image_callback(self, msg):
        """
        Process incoming camera images using TwinLiteNet+ lane extraction while
        keeping the rest of the classic lane_detection.py pipeline intact.
        """
        if self.twinlite_max_inference_hz > 0.0:
            now = time.time()
            min_period = 1.0 / self.twinlite_max_inference_hz
            if now - self._last_infer_time < min_period:
                return
            self._last_infer_time = now

        self.get_logger().info('Image callback received', throttle_duration_sec=5.0)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            lines, lane_mask, edges = self.detect_white_lanes(cv_image)

            if self._latest_da_mask is not None:
                mask_msg = self.bridge.cv2_to_imgmsg(
                    self._latest_da_mask,
                    encoding='mono8'
                )

                # The mask corresponds to the left camera image.
                mask_msg.header = msg.header

                self.drivable_mask_pub.publish(mask_msg)

            viz_image = cv_image.copy()
            detection_image = np.zeros_like(cv_image)

            now = self.get_clock().now()

            # A valid detection occurred.
            if lines is not None and len(lines) > 0:
                self.latest_lines = list(lines)
                self.last_valid_lines = list(lines)
                self.last_valid_line_time = now

            # Detection failed for this frame.
            else:
                time_since_valid = (
                    now - self.last_valid_line_time
                ).nanoseconds / 1e9

                # Keep publishing the previous lanes briefly instead of blinking.
                if self.last_valid_lines and time_since_valid < self.lane_hold_time:
                    self.latest_lines = list(self.last_valid_lines)

                    self.get_logger().warning(
                        'No lanes this frame; holding previous lane markers',
                        throttle_duration_sec=1.0
                    )
                else:
                    self.latest_lines = []

            # Draw whichever lines will actually be published.
            for line in self.latest_lines:
                x1, y1, x2, y2 = line

                cv2.line(
                    detection_image,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                cv2.line(
                    viz_image,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    3
                )
                # cv2.line(detection_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # cv2.line(viz_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

                # self.get_logger().info(f'Detected {len(lines)} lane segments', throttle_duration_sec=2.0)
            self.get_logger().info(
                f'Publishing {len(self.latest_lines)} lane segments',
                throttle_duration_sec=2.0
            )

            if self.show_viz:
                lane_mask_colored = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)

                # Dedicate the bottom-right panel to the drivable-area mask itself
                # (not just the overlay) so it's unambiguous that TwinLiteNet+ is
                # actually producing a drivable-area prediction, not just lane lines.
                if self._latest_da_mask is not None:
                    da_panel = np.zeros_like(cv_image)
                    da_panel[:, :, 0] = self._latest_da_mask  # blue, matches the overlay tint
                    bottom_right_label = 'DRIVABLE AREA (TWINLITENET+)'
                else:
                    da_panel = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                    bottom_right_label = 'EDGES (drivable area disabled)'

                left_view = self.draw_drivable_area_overlay(viz_image, self._latest_da_mask)
                right_view = self.latest_right_image.copy() if self.latest_right_image is not None else np.zeros_like(cv_image)

                stereo_row = np.hstack([left_view, right_view])
                processing_row = np.hstack([lane_mask_colored, da_panel])
                combined = np.vstack([stereo_row, processing_row])
                combined_resized = cv2.resize(combined, (1280, 720))

                cv2.putText(combined_resized, 'LIVE STEREO: LEFT (+ drivable area overlay)', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(combined_resized, 'LIVE STEREO: RIGHT', (650, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(combined_resized, 'TWINLITENET+ LANE MASK', (10, 390),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(combined_resized, bottom_right_label, (650, 390),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
                cv2.putText(combined_resized, f'DETECTED SEGMENTS: {len(lines) if lines is not None else 0}',
                            (10, 430),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                device_label = self.twinlite_device if self.twinlite_device else 'n/a'
                cv2.putText(combined_resized, f'VARIANT: {self.twinlite_variant.upper()} ({device_label})',
                            (650, 430),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

                # Status bar: make it unmistakable when the model failed to load and
                # this is silently running the classic detector instead (e.g. missing
                # .pth checkpoint) -- a quiet "(None)" device label is easy to miss.
                status_bar_h = 36
                canvas = np.zeros((720 + status_bar_h, 1280, 3), dtype=np.uint8)
                canvas[:720, :, :] = combined_resized
                if self.twinlite_model_ready:
                    bar_color = (0, 120, 0)
                    status_text = f'TWINLITENET+ MODEL ACTIVE - variant={self.twinlite_variant} device={self.twinlite_device}'
                else:
                    bar_color = (0, 0, 200)
                    status_text = 'FALLBACK: TwinLiteNet+ weights NOT loaded -- running classic detector (check terminal log)'
                cv2.rectangle(canvas, (0, 720), (1280, 720 + status_bar_h), bar_color, -1)
                cv2.putText(canvas, status_text, (12, 720 + 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)

                cv2.imshow('Lane Detection - ESDA STEREO (TwinLiteNet+)', canvas)
                cv2.waitKey(1)

            if len(self.latest_lines) > 0:
                lines_array = [[line] for line in self.latest_lines]

                from std_msgs.msg import Header

                header = Header()
                header.frame_id = 'camera_link_optical'
                header.stamp = self.get_clock().now().to_msg()

                self.publish_lane_markers(
                    lines_array,
                    header
                )

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
    node = LaneDetectionTwinLiteNode()

    # MultiThreadedExecutor (paired with the ReentrantCallbackGroup on image_sub)
    # so a slow inference pass never starves scan_callback's /scan_fused republish.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

