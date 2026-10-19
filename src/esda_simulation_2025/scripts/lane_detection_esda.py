#!/usr/bin/env python3
"""
lane_detection_esda.py -- a fourth detector for the LaneDetectionNode family.

Same deal as lane_detection_FCN.py and lane_detection_twinlite.py: it
subclasses the base node and overrides ONLY detect_white_lanes. The depth
projection, /lane_markers, /lane_obstacles and the /scan_fused injection all
stay in the base class, untouched, so the waypoint navigator sees exactly
what it sees today.

WHAT IT CHANGES, AND WHY

The base class thresholds grayscale at a fixed level (white_threshold_low,
default 130). That works when the ground is dark and evenly lit. It has two
failure modes outdoors, both measured on a 395-frame benchmark where the
camera was teleported to known poses so the ground truth came from world
geometry rather than from odometry:

  * A FIXED level cannot follow the light. Scored across four lightings --
    clean, brightened, darkened, a gradient and a hard shadow band -- a
    global threshold held 0.800 recall clean but fell to 0.678 under the
    shadow, and its precision collapsed from 0.452 to 0.178 under the
    gradient. An ADAPTIVE threshold, which compares each pixel with its
    local neighbourhood, held 0.791-0.795 recall and 0.814-0.825 at 1-2 m
    across every one of those conditions. Outdoors, shade and gradients are
    the normal case, so that flatness is worth more than the clean-frame
    peak.

  * A GLOBAL threshold cannot decline. Otsu in particular always splits the
    histogram in two, even when the frame contains only one thing, so bare
    tarmac with no paint in it gets split into "darker tarmac" and "lighter
    tarmac" and the lighter half is reported as lane. On the IGVC course
    that filled the costmap and drove the robot off the track. Hence
    max_area_frac below: real paint measured 0.7%-3.5% of the region of
    interest, so a mask covering more than a sixth of it is not a lane
    whatever the threshold thinks, and is dropped entirely.

WHAT IT DELIBERATELY DOES NOT CHANGE

The tail of detect_white_lanes -- Canny, HoughLinesP, filter_valid_lanes --
is the base class's, called unchanged with the base class's own parameters.
Only the mask handed to it differs. That keeps the dark-white-dark profile
check, the orange-cone rejection and the line semantics the navigator was
tuned against, and it makes this an honest A/B: one variable.

The region of interest is also the base class's (the lower half of the
frame). An earlier version of this code derived the ROI from a camera
projection, which is correct but ties the detector to one specific lens and
mount. It does not belong in a file that has to run on whatever camera the
robot carries.

USAGE

    ros2 run esda_simulation_2025 lane_detection_esda.py \
        --ros-args -p detector_method:=adaptive

    python3 lane_detection_esda.py --selftest     # no ROS needed

detector_method: adaptive | otsu | tophat | value | fixed | yolo
  adaptive  local mean. The default, and the one the lighting sweep favours.
  otsu      per-frame global level. Best on clean, evenly lit frames.
  tophat    morphological; best recall at 1-2 m but the worst precision,
            because tarmac speckle narrower than the kernel survives it.
  value     bright AND desaturated in HSV. Collapses in shade; kept for
            completeness, not recommended.
  fixed     the base class's own behaviour, for a baseline.
  yolo      a trained segmentation model. Measured 0.492 recall against
            0.795 for adaptive on the same frames, at 8.4 ms against 2.4,
            so it is NOT the default -- but the weights were trained for a
            different camera, so this is the hook to re-test after a
            retrain rather than a recommendation.
"""

import argparse
import sys

import cv2
import numpy as np

METHODS = ("adaptive", "otsu", "tophat", "value", "fixed", "yolo")


# ==========================================================================
# Pure mask extraction.
#
# Module-level and free of rclpy, cv_bridge and the node, so the self-test
# below runs on any machine with numpy and cv2 -- the same reason
# tools/validate_waypoint_logic.py can exercise the clearance maths without
# ROS, Gazebo or Nav2.
# ==========================================================================

def roi_mask_lower(height, width, top_frac=0.5):
    """The base class's region of interest: everything below top_frac."""
    m = np.zeros((height, width), np.uint8)
    m[int(height * top_frac):, :] = 255
    return m


def raw_mask(image, method, white_low=130, adaptive_block=61, adaptive_c=-8,
             tophat_px=25, tophat_thresh=12, v_min=170, s_max=60,
             yolo_mask_fn=None):
    """Binary 0/255 mask of candidate paint, before ROI and cleanup."""
    if method not in METHODS:
        raise ValueError(f"method must be one of {METHODS}, got {method!r}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if method == "fixed":
        _, m = cv2.threshold(gray, white_low, 255, cv2.THRESH_BINARY)
    elif method == "otsu":
        _, m = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    elif method == "adaptive":
        blk = int(adaptive_block) | 1          # must be odd
        m = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv2.THRESH_BINARY, blk, adaptive_c)
    elif method == "tophat":
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (tophat_px, tophat_px))
        th = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, k)
        m = np.where(th >= tophat_thresh, 255, 0).astype(np.uint8)
    elif method == "value":
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        m = np.where((hsv[..., 2] >= v_min) & (hsv[..., 1] <= s_max),
                     255, 0).astype(np.uint8)
    else:                                       # yolo
        if yolo_mask_fn is None:
            raise RuntimeError("method 'yolo' needs a yolo_mask_fn")
        m = (np.asarray(yolo_mask_fn(image)).astype(bool) * 255).astype(np.uint8)
    return m


def clean_mask(mask, roi, min_area=120, close_px=3, max_area_frac=0.15):
    """ROI, morphology, speck removal, and the flood guard.

    The flood guard is the important one and it is not optional: a global
    threshold applied to a frame with no paint in it reports a large fraction
    of the ground as lane, and downstream that becomes obstacles. Real paint
    measured 0.7%-3.5% of the ROI, so anything over max_area_frac is refused
    outright rather than scaled back -- a partly-wrong lane is worse than no
    lane, because the navigator cannot tell which part.
    """
    m = cv2.bitwise_and(mask, roi)
    if close_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    k3 = np.ones((3, 3), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k3)
    if min_area > 0:
        n, lab, stats, _ = cv2.connectedComponentsWithStats((m > 0).astype(np.uint8), 8)
        keep = np.zeros(n, bool)
        for i in range(1, n):
            keep[i] = stats[i, cv2.CC_STAT_AREA] >= min_area
        m = (keep[lab] * 255).astype(np.uint8)
    roi_px = int((roi > 0).sum())
    if max_area_frac > 0.0 and roi_px > 0:
        if int((m > 0).sum()) > max_area_frac * roi_px:
            return np.zeros_like(m)
    return m


# ==========================================================================
# Self-test
# ==========================================================================

def selftest():
    print("=" * 70)
    print("lane_detection_esda -- mask extraction, no ROS required")
    print("=" * 70)
    fails = []

    def ck(name, cond):
        print("  %-58s %s" % (name, "ok" if cond else "FAIL"))
        if not cond:
            fails.append(name)

    H, W = 480, 640
    roi = roi_mask_lower(H, W)
    ck("ROI is the lower half", int((roi > 0).sum()) == (H // 2) * W)

    def scene(bg, line, width=9):
        img = np.full((H, W, 3), bg, np.uint8)
        img[:, W // 2 - width // 2:W // 2 + width // 2 + 1] = line
        return img

    lo, hi = W // 2 - 4, W // 2 + 5
    for mth in ("fixed", "otsu", "adaptive", "tophat", "value"):
        m = clean_mask(raw_mask(scene(60, 230), mth), roi)
        ck(f"{mth}: finds a bright line on dark ground",
           (m[H // 2:, lo:hi] > 0).mean() > 0.8)
        ck(f"{mth}: leaves the empty ground alone",
           (m[H // 2:, :200] > 0).mean() < 0.02)
        ck(f"{mth}: reports nothing above the ROI", not (m[:H // 2] > 0).any())

    # Lighting. A fixed level is the one that cannot follow it.
    for bg, line in ((40, 200), (120, 255), (150, 235)):
        for mth in ("adaptive", "tophat"):
            m = clean_mask(raw_mask(scene(bg, line), mth), roi)
            ck(f"{mth}: still finds the line at background {bg}",
               (m[H // 2:, lo:hi] > 0).mean() > 0.8)
    m = clean_mask(raw_mask(scene(150, 235), "fixed", white_low=130), roi)
    ck("fixed: floods when the ground is brighter than its level, "
       "and the guard catches it", not (m > 0).any())

    # The flood guard, which is why this file exists.
    rng = np.random.default_rng(0)
    bare = rng.normal(120, 6, (H, W, 3)).clip(0, 255).astype(np.uint8)
    for mth in ("fixed", "otsu", "adaptive", "tophat", "value"):
        ck(f"{mth}: bare ground with no paint gives nothing",
           not (clean_mask(raw_mask(bare, mth), roi) > 0).any())
    ck("otsu WITHOUT the guard would have flooded",
       (clean_mask(raw_mask(bare, "otsu"), roi, max_area_frac=0.0) > 0).mean() > 0.1)

    # Shapes and contracts.
    m = clean_mask(raw_mask(scene(60, 230), "adaptive"), roi)
    ck("mask is uint8", m.dtype == np.uint8)
    ck("mask is 0/255 only", set(np.unique(m)).issubset({0, 255}))
    ck("mask matches the image shape", m.shape == (H, W))
    try:
        raw_mask(scene(60, 230), "nope")
        ck("an unknown method raises", False)
    except ValueError:
        ck("an unknown method raises", True)
    try:
        raw_mask(scene(60, 230), "yolo")
        ck("yolo without a model raises", False)
    except RuntimeError:
        ck("yolo without a model raises", True)
    ck("yolo path uses the supplied mask",
       (clean_mask(raw_mask(scene(60, 230), "yolo",
                            yolo_mask_fn=lambda im: np.zeros((H, W), bool)),
                   roi) > 0).sum() == 0)

    print()
    if fails:
        print(f"{len(fails)} FAILED: " + ", ".join(fails))
        return 1
    print("ALL CHECKS PASSED")
    print("=" * 70)
    return 0


# ==========================================================================
# The node. ROS imports stay below the self-test so the maths above can be
# exercised on a machine with no ROS on it.
# ==========================================================================

def _build_node_class():
    import rclpy
    from lane_detection import LaneDetectionNode

    class LaneDetectionEsdaNode(LaneDetectionNode):
        def __init__(self):
            super().__init__()
            self.declare_parameter('detector_method', 'adaptive')
            self.declare_parameter('max_area_frac', 0.15)
            self.declare_parameter('adaptive_block', 61)
            self.declare_parameter('adaptive_c', -8)
            self.declare_parameter('tophat_px', 25)
            self.declare_parameter('tophat_thresh', 12)
            self.declare_parameter('min_blob_area', 120)
            self.declare_parameter('yolo_weights', '')

            self.method = str(self.get_parameter('detector_method').value).lower()
            if self.method not in METHODS:
                self.get_logger().error(
                    f"detector_method '{self.method}' unknown, falling back to "
                    f"'adaptive'. Valid: {', '.join(METHODS)}")
                self.method = 'adaptive'
            self.max_area_frac = float(self.get_parameter('max_area_frac').value)
            self._yolo = None
            if self.method == 'yolo':
                self._load_yolo()
            self.get_logger().info(
                f"lane_detection_esda: method={self.method} "
                f"max_area_frac={self.max_area_frac}")

        def _load_yolo(self):
            w = str(self.get_parameter('yolo_weights').value)
            if not w:
                self.get_logger().error(
                    "detector_method:=yolo but yolo_weights is empty -- "
                    "falling back to 'adaptive'")
                self.method = 'adaptive'
                return
            try:
                from ultralytics import YOLO
                self._yolo = YOLO(w)
                self.get_logger().info(f"loaded {w}")
            except Exception as e:                       # noqa: BLE001
                self.get_logger().error(
                    f"could not load {w}: {e} -- falling back to 'adaptive'")
                self.method = 'adaptive'

        def _yolo_mask(self, image):
            res = self._yolo.predict(image, verbose=False)[0]
            h, w = image.shape[:2]
            out = np.zeros((h, w), bool)
            if res.masks is None:
                return out
            names = (res.model.names if hasattr(res, "model")
                     else getattr(self._yolo, "names", {}))
            ids = res.boxes.cls.cpu().numpy().astype(int) if res.boxes is not None else []
            for i, cid in enumerate(ids):
                if str(names.get(int(cid), cid)).lower() != "lane":
                    continue
                mk = res.masks.data[i].cpu().numpy()
                out |= cv2.resize(mk, (w, h), interpolation=cv2.INTER_NEAREST) > 0.5
            return out

        def detect_white_lanes(self, image):
            """Same return contract as the base: (valid_lines, mask, edges).

            Only the MASK is ours. Canny, Hough and filter_valid_lanes below
            are the base class's, called with the base class's parameters, so
            the lines reaching the navigator keep their existing meaning and
            this stays a one-variable change.
            """
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            height, width = gray.shape
            hsv = (cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                   if self.reject_orange_cones else None)

            m = raw_mask(
                image, self.method,
                white_low=self.white_low,
                adaptive_block=int(self.get_parameter('adaptive_block').value),
                adaptive_c=int(self.get_parameter('adaptive_c').value),
                tophat_px=int(self.get_parameter('tophat_px').value),
                tophat_thresh=int(self.get_parameter('tophat_thresh').value),
                yolo_mask_fn=self._yolo_mask if self._yolo is not None else None)

            white_mask = clean_mask(
                m, roi_mask_lower(height, width),
                min_area=int(self.get_parameter('min_blob_area').value),
                max_area_frac=self.max_area_frac)

            if not white_mask.any():
                # Say so once in a while: a detector that has declined is a
                # different situation from one that has not run, and the
                # difference matters when the robot stops moving.
                self.get_logger().debug(
                    "no lane mask this frame (nothing found, or the flood "
                    "guard refused it)")
                return [], white_mask, np.zeros_like(white_mask)

            edges = cv2.Canny(white_mask, 50, 150)
            lines = cv2.HoughLinesP(
                edges, rho=1, theta=np.pi / 180, threshold=15,
                minLineLength=self.min_line_length,
                maxLineGap=self.max_line_gap)
            valid = (self.filter_valid_lanes(lines, gray, height, width, hsv)
                     if lines is not None else [])
            return valid, white_mask, edges

    return rclpy, LaneDetectionEsdaNode


def main(args=None):
    rclpy, cls = _build_node_class()
    rclpy.init(args=args)
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument('--selftest', action='store_true')
    known, _ = ap.parse_known_args()
    if known.selftest:
        sys.exit(selftest())
    main()
