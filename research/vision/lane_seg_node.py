#!/usr/bin/env python3
"""
ESDA lane/cone segmentation node (sim-first, same node runs on the real robot).

Pipeline per frame:
    /head_camera/images  ->  YOLOv8-seg  ->  {lane, cone, drivable} masks
        lane  mask  -> ground-projected PointCloud2  on /lane/points
        cone  boxes -> ground point at box base      on /cone/points
        overlay     -> /lane/debug_image  (RViz/Foxglove)

WHY this exists: the classical white-threshold detector tags white-striped
cones AS lane, which inflates the costmap into a false wall. Here lane and
cone are separate learned classes on separate topics, so they feed different
costmap layers and can never be confused.

Flexible: set classes.mode in vision_config.yaml to full / lane_cone /
lane_only (or --mode). The model still outputs every class; muted classes are
just not published. Good for trimming compute when the GPU/Jetson is loaded.

Run (ROS 2 sourced):
    python3 lane_seg_node.py
    python3 lane_seg_node.py --config vision_config.yaml --mode lane_only

Deps:  pip install ultralytics ;  sudo apt install python3-opencv
"""
import argparse
import os
import struct
import sys
import time

import math
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField, LaserScan
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

# lane_link lives next to this file
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import lane_link
    HAVE_LINK = True
except Exception:
    HAVE_LINK = False

try:
    import cv2
    HAVE_CV = True
except Exception:
    HAVE_CV = False


# ---------------------------------------------------------------------------
#  sensor_msgs/Image <-> numpy, WITHOUT cv_bridge.
#
#  cv_bridge is a C extension built against NumPy 1.x. Installing a CUDA torch
#  pulls in NumPy 2.x, and the ABI mismatch makes the very first conversion
#  die with:
#
#      A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6
#      AttributeError: _ARRAY_API not found   (in cv_bridge.cvtColor2)
#
#  The alternative -- pinning numpy<2 -- would put the verified CUDA torch
#  build at risk to satisfy a conversion we do not need. For the encodings the
#  gz bridge actually emits this is a reshape and at most a channel swap, so
#  doing it here removes the dependency and a copy with it.
# ---------------------------------------------------------------------------
def imgmsg_to_bgr(msg):
    """sensor_msgs/Image -> HxWx3 uint8 BGR."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    nch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if nch is None:
        raise ValueError("unsupported image encoding: %s" % msg.encoding)
    # step is BYTES PER ROW and may include padding, so honour it rather than
    # assuming width*channels.
    row = msg.step if msg.step else msg.width * nch
    a = buf[:msg.height * row].reshape(msg.height, row)
    a = a[:, :msg.width * nch].reshape(msg.height, msg.width, nch)
    if enc == "mono8":
        return cv2.cvtColor(a[:, :, 0], cv2.COLOR_GRAY2BGR)
    a = a[:, :, :3]
    if enc.startswith("rgb"):
        a = a[:, :, ::-1]
    return np.ascontiguousarray(a)


def bgr_to_imgmsg(img, header, encoding="bgr8"):
    """HxWx3 BGR (or HxW mono8) -> sensor_msgs/Image."""
    m = Image()
    m.header = header
    m.height, m.width = img.shape[:2]
    m.encoding = encoding
    m.is_bigendian = 0
    m.step = m.width * (1 if encoding == "mono8" else 3)
    m.data = np.ascontiguousarray(img).tobytes()
    return m


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def scan_to_frame(msg, tf, target):
    """LaserScan -> Nx2 (x, y) in `target`. Empty if TF is not ready."""
    try:
        t = tf.lookup_transform(target, msg.header.frame_id, rclpy.time.Time())
    except Exception:                                   # noqa: BLE001
        return np.empty((0, 2), np.float32)
    r = np.asarray(msg.ranges, dtype=np.float32)
    if r.size == 0:
        return np.empty((0, 2), np.float32)
    ang = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment
    keep = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
    r, ang = r[keep], ang[keep]
    if r.size == 0:
        return np.empty((0, 2), np.float32)
    x, y = r * np.cos(ang), r * np.sin(ang)
    th = _yaw_of(t.transform.rotation)
    c, sn = math.cos(th), math.sin(th)
    return np.stack([t.transform.translation.x + c * x - sn * y,
                     t.transform.translation.y + sn * x + c * y],
                    axis=1).astype(np.float32)


def cluster_xy(pts, eps):
    """Greedy single-link clustering of Nx2 points. -> list of Nx2 arrays.

    Deliberately simple: a barrel contributes a handful of contact points a
    few centimetres apart, so anything more sophisticated earns nothing here.
    """
    out = []
    if len(pts) == 0:
        return out
    unused = np.ones(len(pts), bool)
    for i in range(len(pts)):
        if not unused[i]:
            continue
        idx = [i]
        unused[i] = False
        k = 0
        while k < len(idx):
            d = np.hypot(pts[:, 0] - pts[idx[k], 0], pts[:, 1] - pts[idx[k], 1])
            near = np.where(unused & (d <= eps))[0]
            unused[near] = False
            idx.extend(near.tolist())
            k += 1
        out.append(pts[idx])
    return out


def filled_disc(cx, cy, radius, step):
    """Solid disc of points, so the costmap gets a blob and not a dotted ring.

    The ground-contact edge is a thin arc. At 0.05 m costmap resolution that
    marks a line of cells with gaps between them, and a planner will happily
    thread a path through the gaps -- which is exactly the "it's a dot and it
    tries to pass" behaviour. Filling the barrel's footprint closes them.
    """
    n = max(1, int(radius / step))
    g = np.arange(-n, n + 1) * step
    gx, gy = np.meshgrid(g, g)
    m = (gx * gx + gy * gy) <= radius * radius
    return np.stack([cx + gx[m], cy + gy[m]], axis=1).astype(np.float32)


def solidify_cones(pts3d, scan_xy, gate_on, confirm_r, cluster_eps,
                   disc_r, disc_step):
    """Cone contact points -> confirmed, solid obstacles.

    Two jobs:
      * CONFIRM. A vision-only cone is a guess; the model has false positives
        on shadows and dark boards. If the lidar sees nothing within
        confirm_r of the cluster, drop it. The lidar cannot be fooled by
        appearance, so agreement between the two is a much stronger claim
        than either alone.
      * SOLIDIFY. Replace each surviving cluster with a filled disc at its
        centre so the costmap marks a blob rather than a dotted arc.

    Returns (points Nx3, n_clusters_kept, n_clusters_dropped).
    """
    if len(pts3d) == 0:
        return np.empty((0, 3), np.float32), 0, 0
    kept, dropped = [], 0
    for c in cluster_xy(pts3d[:, :2], cluster_eps):
        cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
        if gate_on:
            if len(scan_xy) == 0:
                dropped += 1
                continue
            d = np.hypot(scan_xy[:, 0] - cx, scan_xy[:, 1] - cy)
            if float(d.min()) > confirm_r:
                dropped += 1
                continue
        kept.append(filled_disc(cx, cy, disc_r, disc_step))
    if not kept:
        return np.empty((0, 3), np.float32), 0, dropped
    xy = np.concatenate(kept, axis=0)
    return (np.stack([xy[:, 0], xy[:, 1], np.zeros(len(xy), np.float32)], axis=1),
            len(kept), dropped)


def load_config(path):
    path = os.path.expanduser(path)
    with open(path) as f:
        return yaml.safe_load(f)


class LaneSegNode(Node):
    def __init__(self, cfg, mode_override=None):
        super().__init__("lane_seg_node")
        self.cfg = cfg
        if not HAVE_CV:
            self.get_logger().error(
                "cv2 missing.  sudo apt install -y python3-opencv")
            raise SystemExit(1)

        # ---- resolve active classes (the flexible bit) ----
        names = cfg["classes"]["names"]
        mode = mode_override or cfg["classes"].get("mode", "full")
        active = cfg["classes"]["modes"].get(mode)
        if active is None:
            self.get_logger().warn(f"unknown mode '{mode}', using all classes")
            active = names
        self.names = names
        self.active = set(active)
        self.name_to_id = {n: i for i, n in enumerate(names)}
        self.get_logger().info(f"mode={mode}  active classes={sorted(self.active)}")

        # ---- which detector ----
        #
        # "classic" runs vision/lane_classic.py -- a morphological top-hat and
        # a threshold -- instead of the network. Painted lines are bright,
        # thin and high contrast, which is the one thing a threshold is good
        # at, and the IGVC AutoNav 2025 winner's own debug view is a binary
        # image rather than a segmentation mask.
        #
        # It matters on this robot specifically. Asterius is an Orin NX
        # carrying a ZED 2i, and ZED depth is GPU work on the same GPU a
        # network would use -- NEURAL depth alone is 20 Hz at ~26 W on this
        # class of board. A threshold leaves that budget to the camera. It
        # also cannot go stale: its ROI is derived from the projection at
        # runtime, so moving the mount does not invalidate it, which is
        # exactly what happened to the weights twice this week.
        #
        # Everything downstream -- projection, linking, the side check, the
        # carrot -- is untouched. Only the thing that produces the mask
        # changes, and the default stays YOLO until the bench says otherwise.
        dcfg = cfg.get("detector", {})
        self.detector = str(dcfg.get("kind", "yolo")).lower()
        self.classic_kw = dict(
            method=str(dcfg.get("method", "tophat")),
            min_area=int(dcfg.get("min_area", 120)),
            close_px=int(dcfg.get("close_px", 3)),
            tophat_px=int(dcfg.get("tophat_px", 25)),
            tophat_thresh=int(dcfg.get("tophat_thresh", 12)),
            # The classic ROI stops where the projection stops, so the
            # detector cannot report paint the stack would throw away.
            max_range=float(cfg.get("projection", {}).get("max_range", 6.0)))
        self.classic = None
        if self.detector == "classic":
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            import lane_classic
            self.classic = lane_classic
            self.model = None
            self.get_logger().info(
                "detector: lane_classic method=%s (no weights, no GPU)"
                % self.classic_kw["method"])
            if "cone" in self.active or "drivable" in self.active:
                self.get_logger().warn(
                    "the classic detector only produces LANE. Active classes "
                    "%s include others, which will stay empty -- the lidar "
                    "owns cones." % sorted(self.active))

        # ---- load model ----
        mcfg = cfg["model"]
        weights = os.path.expanduser(mcfg["weights"])
        if self.detector == "classic":
            pass                      # no network to load
        elif not os.path.isfile(weights):
            self.model = None
            self.get_logger().error(
                f"weights not found: {weights}\n"
                f"  Train first (vision/train.py) or set model.weights in the config.\n"
                f"  Running in PASS-THROUGH mode: debug image only, no detections.")
        else:
            try:
                from ultralytics import YOLO
                self.model = YOLO(weights)
                self.conf = float(mcfg.get("conf", 0.35))
                self.iou = float(mcfg.get("iou", 0.5))
                self.imgsz = int(mcfg.get("imgsz", 640))
                self.half = bool(mcfg.get("half", False))
                self.device = str(mcfg.get("device", "0"))
                if self.device != "cpu":
                    try:
                        import torch
                        if not torch.cuda.is_available():
                            self.get_logger().warn(
                                f"config asks for device '{self.device}' but no CUDA is "
                                f"visible -- falling back to cpu")
                            self.device = "cpu"
                            self.half = False   # fp16 is a GPU-only path
                    except Exception:
                        self.device = "cpu"
                        self.half = False
                self.get_logger().info(f"loaded {weights} on device {self.device}")
            except Exception as e:
                self.get_logger().error(f"failed to load model: {e}")

        # ---- projection params ----
        self.proj = cfg.get("projection", {})
        self.out_frame = cfg["frames"]["output_frame"]
        self.routing = cfg["routing"]

        # ---- pubs ----
        t = cfg["topics"]
        self.pub_debug = self.create_publisher(Image, t["debug_image"], 1)
        self.pub_mask = self.create_publisher(Image, t["lane_mask"], 1)
        self.cloud_pubs = {}
        for cls, r in self.routing.items():
            if r.get("publish_points"):
                self.cloud_pubs[cls] = self.create_publisher(PointCloud2, r["cloud_topic"], 1)

        # ---- lane linking --------------------------------------------------
        # A dashed or barrel-occluded lane reaches the costmap as separate
        # blobs, and Nav2 will plan straight through the holes between them.
        # Linking rebuilds each boundary as one continuous line.
        self.link_cfg = dict(cfg.get("linking", {}) or {})
        self.link_on = bool(self.link_cfg.get("enabled", True)) and HAVE_LINK
        if self.link_cfg.get("enabled", True) and not HAVE_LINK:
            self.get_logger().warn(
                "linking requested but lane_link.py could not be imported -- "
                "publishing raw lane points, so dashed lanes will leave gaps "
                "in the costmap")
        self.pub_raw = None
        self.pub_lines = None
        if self.link_on:
            self.pub_raw = self.create_publisher(
                PointCloud2, self.link_cfg.get("raw_topic", "/lane/points_raw"), 1)
            self.pub_lines = self.create_publisher(
                MarkerArray, self.link_cfg.get("marker_topic", "/lane/lines"), 1)
            self.get_logger().info("lane linking ON -> continuous /lane/points")
        self._last_link = None

        # ---- cone gating by lidar ----
        # A vision cone is a guess; a lidar return is a measurement. Marking
        # the costmap only where the two agree removes the shadow/board false
        # positives without weakening the thing that matters -- the cone CLASS
        # still keeps barrels out of the lane mask whether or not it is
        # confirmed here.
        self.gate = cfg.get("cone_gate", {}) or {}
        self.gate_on = bool(self.gate.get("enabled", True))
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self._scan_xy = np.empty((0, 2), np.float32)
        self.create_subscription(
            LaserScan, t.get("scan_in", "/scan"),
            self._on_scan, qos_profile_sensor_data)

        # ---- sub ----
        self.create_subscription(Image, t["image_in"], self.cb, qos_profile_sensor_data)
        self.get_logger().info(f"subscribed {t['image_in']}  ->  debug {t['debug_image']}")

        # throttle
        self._last = self.get_clock().now()
        self._min_dt = 1.0 / float(cfg.get("runtime", {}).get("rate_hz", 5.0))

        # ---- throughput meter ----
        #
        # WHY THIS EXISTS, in one measurement: across three scored runs the
        # carrot count, the accepted-goal count and the number of times this
        # node published /lane/points were the SAME number -- 42/42, 20/20,
        # 22/22. Distance travelled is therefore set by nothing except how
        # many camera frames arrive, and 42 frames in 305 s of simulated time
        # is one frame per 29 s of wall clock from a sensor asked for 5 Hz.
        #
        # "Slow" is not a finding you can act on. Which stage is slow is. So
        # this prints, on a fixed simulated-time tick: frames that arrived,
        # frames the throttle dropped, frames published, how long inference
        # took, how long the whole callback took, and how much wall clock a
        # simulated second is costing. A run where frames_in is 2 and
        # inference is 20 ms says the simulator is starving the camera; one
        # where frames_in is 50 and inference is 900 ms says the model is.
        self._n_in = 0
        self._n_throttled = 0
        self._n_pub = 0
        self._ms_infer = 0.0
        self._ms_total = 0.0
        self._stats_every = float(cfg.get("runtime", {}).get("stats_every_s", 10.0))
        self._stamp_first = None     # sim time of the first frame this window
        self._stamp_last = None      # ... and of the last
        self._stat_wall0 = time.monotonic()
        if self._stats_every > 0:
            self.create_timer(self._stats_every, self._report_rate)

    # -----------------------------------------------------------------
    def _report_rate(self):
        wall_s = time.monotonic() - self._stat_wall0
        self._stat_wall0 = time.monotonic()
        if wall_s <= 0:
            return
        n, t_i, t_a = self._n_in, self._ms_infer, self._ms_total
        # Simulated time comes from the image stamps, which Gazebo writes.
        sim_s = 0.0
        if self._stamp_first is not None and self._stamp_last is not None:
            sim_s = self._stamp_last - self._stamp_first
        rtf = (sim_s / wall_s) if (sim_s > 0 and n > 1) else float("nan")
        per_sim_s = (self._n_pub / sim_s) if sim_s > 0 else float("nan")
        self.get_logger().info(
            "perception: %d frames in, %d published, %d dropped by throttle "
            "| inference %s, callback %s | %.1f s of simulated time in "
            "%.1f s of wall clock (RTF %.3f) -> %.2f publishes per "
            "simulated second, %.2f per wall second"
            % (n, self._n_pub, self._n_throttled,
               ("%.0f ms" % (t_i / n)) if n else "n/a",
               ("%.0f ms" % (t_a / n)) if n else "n/a",
               sim_s, wall_s, rtf, per_sim_s, self._n_pub / wall_s))
        if n == 0:
            self.get_logger().warn(
                "perception: NO camera frames in the last %.1f s of wall "
                "clock. Whatever is limiting the run is upstream of this "
                "node -- the Gazebo sensor render, not the model." % wall_s)
        self._n_in = self._n_throttled = self._n_pub = 0
        self._ms_infer = self._ms_total = 0.0
        self._stamp_first = self._stamp_last = None

    # -----------------------------------------------------------------
    def _on_scan(self, msg):
        self._scan_xy = scan_to_frame(
            msg, self.tf, self.out_frame)

    def cb(self, msg):
        now = self.get_clock().now()
        self._n_in += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._stamp_first is None:
            self._stamp_first = stamp
        self._stamp_last = stamp
        if (now - self._last).nanoseconds * 1e-9 < self._min_dt:
            self._n_throttled += 1
            return
        self._last = now
        t_cb0 = time.monotonic()

        img = self._to_native(imgmsg_to_bgr(msg))
        h, w = img.shape[:2]
        overlay = img.copy()

        if self.classic is None and self.model is None:
            self._publish_debug(overlay, msg.header)
            return

        lane_mask = np.zeros((h, w), np.uint8)
        per_class_pts = {c: [] for c in self.cloud_pubs}       # dense mask pixels
        per_class_exact = {c: [] for c in self.cloud_pubs}     # sparse contact points

        if self.classic is not None:
            # Same output as the network's lane head: a boolean mask, painted
            # into the overlay and handed to _project. Lane paint is flat on
            # the ground, so the whole mask projects correctly -- none of the
            # ground-contact-edge special casing the cone class needs.
            t_inf0 = time.monotonic()
            cmask = self.classic.lane_mask(img, **self.classic_kw)
            self._ms_infer += (time.monotonic() - t_inf0) * 1e3
            if cmask.any():
                overlay[cmask] = (0.5 * overlay[cmask]
                                  + 0.5 * np.array((0, 255, 0))).astype(np.uint8)
                lane_mask[cmask] = 255
                if "lane" in per_class_pts:
                    ys, xs = np.where(cmask)
                    per_class_pts["lane"].append(np.stack([xs, ys], axis=1))
            res = None
            n_det = 0
            masks_data = None
            cls_ids = np.empty(0, int)
            xyxy = np.empty((0, 4))
        else:
            kw = dict(conf=self.conf, iou=self.iou, imgsz=self.imgsz,
                      device=self.device, verbose=False)
            if self.half:                  # fp16 is a GPU path; passing it on
                kw["half"] = True          # cpu just emits a deprecation warning
            t_inf0 = time.monotonic()
            res = self.model.predict(img, **kw)[0]
            self._ms_infer += (time.monotonic() - t_inf0) * 1e3
            n_det = len(res.boxes) if res.boxes is not None else 0
            masks_data = res.masks.data.cpu().numpy() if res.masks is not None else None
            cls_ids = (res.boxes.cls.cpu().numpy().astype(int)
                       if n_det else np.empty(0, int))
            xyxy = res.boxes.xyxy.cpu().numpy() if n_det else np.empty((0, 4))

        colour = {"lane": (0, 255, 0), "cone": (0, 128, 255),
                  "drivable": (255, 128, 0)}

        for i in range(n_det):
            cid = int(cls_ids[i])
            cname = self.names[cid] if cid < len(self.names) else str(cid)
            if cname not in self.active:
                continue

            mask = None
            if masks_data is not None and i < len(masks_data):
                mask = cv2.resize(masks_data[i], (w, h),
                                  interpolation=cv2.INTER_NEAREST) > 0.5
                if not mask.any():
                    mask = None

            if mask is not None:
                col = colour.get(cname, (200, 200, 200))
                overlay[mask] = (0.5 * overlay[mask]
                                 + 0.5 * np.array(col)).astype(np.uint8)
                if cname == "lane":
                    lane_mask[mask] = 255
                if cname in per_class_pts:
                    ys, xs = np.where(mask)
                    if cname == "cone":
                        # A cone/barrel is a VERTICAL object, and _project
                        # intersects each pixel ray with the ground plane. A
                        # pixel a metre up the barrel therefore lands far
                        # BEYOND it along the viewing ray, so projecting the
                        # whole mask smears one barrel into a long streak --
                        # 545 points from a single barrel in the drive log,
                        # and the fan of phantom obstacles in the costmap.
                        #
                        # Only the ground-contact edge projects correctly, so
                        # keep the lowest masked pixel in each column. Lane
                        # paint is flat on the ground, so it is unaffected and
                        # still uses the full mask.
                        order = np.lexsort((ys, xs))
                        xs_s, ys_s = xs[order], ys[order]
                        last = np.ones(len(xs_s), bool)
                        last[:-1] = xs_s[1:] != xs_s[:-1]
                        per_class_pts[cname].append(
                            np.stack([xs_s[last], ys_s[last]], axis=1))
                    else:
                        per_class_pts[cname].append(np.stack([xs, ys], axis=1))

            x1, y1, x2, y2 = xyxy[i]
            cv2.rectangle(overlay, (int(x1), int(y1)), (int(x2), int(y2)),
                          colour.get(cname, (200, 200, 200)), 2)
            cv2.putText(overlay, cname, (int(x1), max(0, int(y1) - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # EVERY mask-less cone contributes its own ground contact point.
            if mask is None and cname == "cone" and "cone" in per_class_exact:
                per_class_exact["cone"].append(
                    np.array([[int((x1 + x2) / 2), int(y2)]]))

        # ---- project each class to ground + publish ----
        self._last_link = None
        for cname, pub in self.cloud_pubs.items():
            parts = []
            chunks = per_class_pts.get(cname, [])
            if chunks:
                parts.append(self._project(np.concatenate(chunks, axis=0), w, h))
            exact = per_class_exact.get(cname, [])
            if exact:
                parts.append(self._project(np.concatenate(exact, axis=0), w, h,
                                           subsample=False))
            parts = [p for p in parts if len(p)]
            if not parts:
                pub.publish(self._empty_cloud(msg.header))
                if cname == "lane" and self.pub_raw is not None:
                    self.pub_raw.publish(self._empty_cloud(msg.header))
                continue
            pts3d = np.concatenate(parts, axis=0)

            if cname == "cone":
                pts3d, nk, nd = solidify_cones(
                    pts3d, self._scan_xy, self.gate_on,
                    float(self.gate.get("confirm_radius", 0.60)),
                    float(self.gate.get("cluster_eps", 0.45)),
                    float(self.gate.get("disc_radius", 0.28)),
                    float(self.gate.get("disc_step", 0.04)))
                if nd:
                    self.get_logger().info(
                        f"cone: {nk} confirmed by lidar, {nd} dropped "
                        f"(no return within "
                        f"{self.gate.get('confirm_radius', 0.60)} m)")
                if not len(pts3d):
                    pub.publish(self._empty_cloud(msg.header))
                    continue

            if cname == "lane" and self.link_on:
                # raw first, for comparison in RViz
                if self.pub_raw is not None:
                    self.pub_raw.publish(self._make_cloud(pts3d, msg.header))
                try:
                    res = lane_link.link_lanes(pts3d[:, :2], self.link_cfg)
                except Exception as e:
                    self.get_logger().error(f"lane linking failed: {e}")
                    pub.publish(self._make_cloud(pts3d, msg.header))
                    continue
                self._last_link = res
                if len(res.dense):
                    dense3 = np.concatenate(
                        [res.dense, np.zeros((len(res.dense), 1), np.float32)], 1)
                    pub.publish(self._make_cloud(dense3.astype(np.float32), msg.header))
                else:
                    pub.publish(self._make_cloud(pts3d, msg.header))
                if self.pub_lines is not None:
                    self.pub_lines.publish(self._lane_markers(res, msg.header))
                continue

            pub.publish(self._make_cloud(pts3d, msg.header))

        # ---- publish lane mask + debug ----
        if "lane" in self.active:
            self.pub_mask.publish(bgr_to_imgmsg(lane_mask, msg.header, "mono8"))
        if self._last_link is not None:
            self._draw_links(overlay, self._last_link)
        self._publish_debug(overlay, msg.header)
        self._n_pub += 1
        self._ms_total += (time.monotonic() - t_cb0) * 1e3

    # -----------------------------------------------------------------
    def _to_native(self, img):
        """Bring an incoming frame to the resolution the intrinsics describe.

        _project() below uses fx/fy/cx/cy straight from vision_config.yaml
        and ignores the image's actual size, so a camera rendering at any
        other resolution puts every projected point in the wrong place --
        silently, with no error anywhere. Rather than carry a second set of
        intrinsics per resolution, bring the image to the intrinsics.

        This is what lets the sim camera drop to 320x240 -- a quarter of the
        pixels, and the renderer is the thing starving this robot -- while
        the model still receives the 640x480-shaped input it was trained on
        and every projection constant stays exactly where it was. Detail is
        lost; geometry is not.
        """
        p = self.proj
        nw = int(round(2.0 * float(p.get("cx", 320.0))))
        nh = int(round(2.0 * float(p.get("cy", 240.0))))
        h, w = img.shape[:2]
        if (w, h) == (nw, nh) or nw < 2 or nh < 2:
            return img
        if not getattr(self, "_said_resize", False):
            self.get_logger().info(
                f"camera delivers {w}x{h}; upscaling to {nw}x{nh} to match "
                f"the projection intrinsics (cx={p.get('cx')}, cy={p.get('cy')})")
            self._said_resize = True
        return cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)

    def _project(self, pixels, w, h, subsample=True):
        """Flat-ground inverse perspective mapping -> Nx3 points in output_frame.

        subsample=False for sparse points (cone ground contacts): the
        row/col stride is only meaningful for dense segmentation masks.
        """
        p = self.proj
        if not p.get("enabled", True):
            return np.empty((0, 3))
        fx, fy = p["fx"], p["fy"]
        cx, cy = p["cx"], p["cy"]
        H = p["cam_height"]
        pitch = np.deg2rad(p.get("cam_pitch_deg", 0.0))
        xoff = p.get("cam_x_offset", 0.0)
        max_r, min_r = p.get("max_range", 6.0), p.get("min_range", 0.25)
        ys, xs = p.get("row_stride", 1), p.get("col_stride", 1)

        px = pixels
        if subsample and (ys > 1 or xs > 1):
            keep = (pixels[:, 1] % ys == 0) & (pixels[:, 0] % xs == 0)
            px = pixels[keep]
        if len(px) == 0:
            return np.empty((0, 3))

        u = px[:, 0].astype(np.float64)
        v = px[:, 1].astype(np.float64)
        # ray in optical frame (z fwd, x right, y down)
        dx = (u - cx) / fx
        dy = (v - cy) / fy
        dz = np.ones_like(dx)
        # apply camera down-pitch about optical x axis
        cp, sp = np.cos(pitch), np.sin(pitch)
        dy2 = cp * dy + sp * dz
        dz2 = -sp * dy + cp * dz
        # intersect ground plane: optical y (down) hits +H
        with np.errstate(divide="ignore", invalid="ignore"):
            t = H / dy2
        valid = (t > 0) & np.isfinite(t)
        t = t[valid]
        dx_v, dz_v = dx[valid], dz2[valid]
        # base_link: X forward = optical z, Y left = -optical x
        X = t * dz_v + xoff
        Y = -t * dx_v
        Z = np.zeros_like(X)
        rng = np.sqrt(X * X + Y * Y)
        m = (rng >= min_r) & (rng <= max_r)
        return np.stack([X[m], Y[m], Z[m]], axis=1).astype(np.float32)

    # -----------------------------------------------------------------
    def _to_pixel(self, XY):
        """Ground (X fwd, Y left) -> image pixel. Inverse of _project().

        Used only to draw the reconstructed lane lines back onto the debug
        image, so you can SEE which fragments were joined.
        """
        p = self.proj
        fx, fy = p["fx"], p["fy"]
        cx, cy = p["cx"], p["cy"]
        Hc = p["cam_height"]
        pitch = np.deg2rad(p.get("cam_pitch_deg", 0.0))
        xoff = p.get("cam_x_offset", 0.0)
        cp, sp = np.cos(pitch), np.sin(pitch)

        XY = np.asarray(XY, np.float64).reshape(-1, 2)
        A = XY[:, 0] - xoff
        denom = A * cp + Hc * sp
        with np.errstate(divide="ignore", invalid="ignore"):
            sray = (Hc * cp - A * sp) / denom
            t = Hc / (cp * sray + sp)
            u = cx + fx * (-XY[:, 1] / t)
            v = cy + fy * sray
        ok = np.isfinite(u) & np.isfinite(v) & (A > 0.05)
        return u, v, ok

    _GROUP_COLOURS = [(0, 255, 0), (255, 200, 0), (255, 0, 255),
                      (0, 255, 255), (160, 255, 160), (255, 160, 80)]

    def _draw_links(self, overlay, res):
        """Paint each reconstructed lane line in its own colour, and mark the
        bridged gaps, so the association is visible at a glance."""
        try:
            for gid, poly in enumerate(res.polylines):
                col = self._GROUP_COLOURS[gid % len(self._GROUP_COLOURS)]
                if poly is None or len(poly) < 2:
                    continue
                u, v, ok = self._to_pixel(poly)
                pts = np.stack([u[ok], v[ok]], 1).astype(np.int32)
                if len(pts) >= 2:
                    cv2.polylines(overlay, [pts], False, col, 2, cv2.LINE_AA)
                    cv2.putText(overlay, f"L{gid}", tuple(pts[len(pts) // 2]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            # dot every fragment centroid, coloured by the line it joined
            for fi, f in enumerate(res.fragments):
                gid = res.group_of_fragment.get(fi, 0)
                col = self._GROUP_COLOURS[gid % len(self._GROUP_COLOURS)]
                u, v, ok = self._to_pixel(f.centroid.reshape(1, 2))
                if ok[0]:
                    cv2.circle(overlay, (int(u[0]), int(v[0])), 3, col, -1)
        except Exception:
            pass   # drawing must never take the node down

    def _lane_markers(self, res, header):
        """MarkerArray of the reconstructed lane lines, for RViz/Foxglove."""
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.out_frame
        clear.header.stamp = header.stamp
        clear.ns = "lane_lines"
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for gid, poly in enumerate(res.polylines):
            if poly is None or len(poly) < 2:
                continue
            m = Marker()
            m.header.frame_id = self.out_frame
            m.header.stamp = header.stamp
            m.ns = "lane_lines"
            m.id = gid
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.06
            c = self._GROUP_COLOURS[gid % len(self._GROUP_COLOURS)]
            m.color = ColorRGBA(r=c[2] / 255.0, g=c[1] / 255.0,
                                b=c[0] / 255.0, a=0.9)
            m.pose.orientation.w = 1.0
            for x, y in poly:
                m.points.append(Point(x=float(x), y=float(y), z=0.0))
            arr.markers.append(m)
        return arr

    def _make_cloud(self, pts, header):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.out_frame
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = pts.tobytes()
        return msg

    def _empty_cloud(self, header):
        return self._make_cloud(np.empty((0, 3), np.float32), header)

    def _publish_debug(self, overlay, header):
        if not self.cfg.get("runtime", {}).get("draw_debug", True):
            return
        self.pub_debug.publish(bgr_to_imgmsg(overlay, header, "bgr8"))


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--config", default=os.path.join(here, "vision_config.yaml"))
    ap.add_argument("--mode", default=None, choices=["full", "lane_cone", "lane_only"],
                    help="override classes.mode from the config")
    # GPU overrides, so you can A/B without editing the config file.
    ap.add_argument("--device", default=None,
                    help="'0' for the first CUDA GPU, 'cpu' to force CPU. "
                         "Overrides model.device.")
    ap.add_argument("--half", dest="half", action="store_true", default=None,
                    help="FP16 inference (GPU only, roughly 2x). Overrides model.half.")
    ap.add_argument("--no-half", dest="half", action="store_false",
                    help="force FP32 even if the config asks for half")
    ap.add_argument("--detector", default=None, choices=["yolo", "classic"],
                    help="'classic' uses lane_classic.py (a threshold, no "
                         "network, no GPU) instead of YOLO. Overrides "
                         "detector.kind.")
    ap.add_argument("--classic-method", default=None,
                    choices=["otsu", "value", "adaptive", "tophat"],
                    help="which threshold, when --detector classic")
    ap.add_argument("--no-link", action="store_true",
                    help="disable lane linking (publish raw lane points)")
    args, _ = ap.parse_known_args()

    cfg = load_config(args.config)
    if args.device is not None:
        cfg.setdefault("model", {})["device"] = args.device
    if args.half is not None:
        cfg.setdefault("model", {})["half"] = bool(args.half)
    if args.detector is not None:
        cfg.setdefault("detector", {})["kind"] = args.detector
    if args.classic_method is not None:
        cfg.setdefault("detector", {})["method"] = args.classic_method
    if args.no_link:
        cfg.setdefault("linking", {})["enabled"] = False
    rclpy.init()
    node = LaneSegNode(cfg, mode_override=args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
