#!/usr/bin/env python3
"""record_combined.py -- one video showing the detector and the costmap.

    python3 vision/record_combined.py --out ~/esda_runs/run.mp4 --seconds 240

Left pane: whatever the lane detector is drawing (/lane/debug_image).
Right pane: the Nav2 costmap it feeds, with the robot footprint on it.

Why both in ONE file rather than two: the interesting question is never
"what did the detector see" or "what did the planner believe" on their own,
it is whether the second followed from the first. Two files with different
frame counts and different start times cannot answer that; two panes on the
same frame can.

THE COSTMAP IS NOT A PLAIN TOPIC. Nav2 publishes the full OccupancyGrid once,
latched (transient-local), and then sends only deltas on
<topic>_updates. Subscribing to the full topic alone with default QoS gets
you either nothing at all or one frozen frame -- which looks like a working
recording of a stationary costmap. So this matches the transient-local QoS
and applies the update patches.

Colours follow the usual costmap reading:
    grey      unknown (-1)
    white     free (0)
    blue->red rising cost (1-98)
    cyan      inscribed (99)
    magenta   lethal (100)
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import cv2
    HAVE_CV = True
except ImportError:
    HAVE_CV = False

UNKNOWN = (60, 60, 60)
FREE = (248, 248, 248)
INSCRIBED = (255, 255, 0)      # BGR cyan-ish
LETHAL = (255, 0, 255)         # BGR magenta


def cost_lut():
    """256-entry BGR lookup, indexed by cost+1 so -1 lands at 0."""
    lut = np.zeros((256, 3), np.uint8)
    lut[0] = UNKNOWN                     # -1
    lut[1] = FREE                        # 0
    for v in range(1, 99):               # 1..98
        t = v / 98.0
        lut[v + 1] = (int(255 * (1 - t)), int(80 * (1 - t)), int(255 * t))
    lut[100] = INSCRIBED                 # 99
    lut[101] = LETHAL                    # 100
    return lut


LUT = cost_lut()


def colourise(grid):
    """int8 OccupancyGrid data as HxW -> BGR image, origin at bottom-left.

    The grid's first row is the origin row, which is the BOTTOM of the map
    in the frame's sense, so it is flipped for display. Getting this wrong
    gives a costmap that is a mirror of the world and still looks
    plausible.
    """
    g = np.asarray(grid, dtype=np.int16)
    idx = np.clip(g + 1, 0, 101).astype(np.uint8)
    img = LUT[idx]
    # ascontiguousarray, not just flipud: flipud returns a NEGATIVE-STRIDE
    # VIEW, and cv2 drawing calls refuse it with "Layout of the output array
    # is incompatible with cv::Mat" -- which surfaces as a crash in the
    # middle of a recording, not at startup.
    return np.ascontiguousarray(np.flipud(img))


def apply_update(grid, x, y, w, h, data):
    """Patch an OccupancyGridUpdate into the full grid, in place.

    Returns False and changes nothing when the patch does not fit, rather
    than raising: a costmap resize mid-run should cost one frame, not the
    whole recording.
    """
    H, W = grid.shape
    if x < 0 or y < 0 or x + w > W or y + h > H or len(data) != w * h:
        return False
    grid[y:y + h, x:x + w] = np.asarray(data, dtype=np.int16).reshape(h, w)
    return True


def label(img, text, scale=0.55):
    cv2.rectangle(img, (0, 0), (img.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(img, text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, scale,
                (255, 255, 255), 1, cv2.LINE_AA)
    return img


def fit(img, h, w):
    """Letterbox into h x w without distorting the aspect ratio."""
    ih, iw = img.shape[:2]
    s = min(h / ih, w / iw)
    r = cv2.resize(img, (max(1, int(iw * s)), max(1, int(ih * s))),
                   interpolation=cv2.INTER_NEAREST)
    out = np.zeros((h, w, 3), np.uint8)
    y0, x0 = (h - r.shape[0]) // 2, (w - r.shape[1]) // 2
    out[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
    return out


def open_writer(path, w, h, fps):
    attempts = [(path, "mp4v")] if path.endswith(".mp4") else []
    attempts.append((os.path.splitext(path)[0] + ".avi", "MJPG"))
    for p, fourcc in attempts:
        vw = cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*fourcc), fps, (w, h))
        if vw.isOpened():
            return vw, p
        vw.release()
    raise SystemExit(f"no usable video encoder for {path}")


def selftest():
    fails = []

    def ck(name, cond):
        print("  %-56s %s" % (name, "ok" if cond else "FAIL"))
        if not cond:
            fails.append(name)

    ck("unknown is grey", tuple(int(v) for v in LUT[0]) == UNKNOWN)
    ck("free is near-white", tuple(int(v) for v in LUT[1]) == FREE)
    ck("inscribed 99 distinct from lethal 100",
       tuple(LUT[100]) != tuple(LUT[101]))
    ck("cost rises monotonically in red", all(
        int(LUT[v + 1][2]) <= int(LUT[v + 2][2]) for v in range(1, 97)))

    g = np.array([[0, 0], [100, -1]], np.int16)
    img = colourise(g)
    ck("colourise keeps the shape", img.shape == (2, 2, 3))
    ck("row 0 of the grid lands at the BOTTOM of the image",
       tuple(int(v) for v in img[1, 0]) == FREE)
    ck("lethal survives the flip",
       tuple(int(v) for v in img[0, 0]) == LETHAL)
    ck("the flipped image is contiguous, so cv2 can draw on it",
       img.flags["C_CONTIGUOUS"])
    if HAVE_CV:
        try:
            cv2.polylines(img, [np.array([[0, 0], [1, 0], [1, 1]], np.int32)],
                          True, (0, 200, 0), 1)
            drew = True
        except Exception:
            drew = False
        ck("cv2 really will draw on a colourise() result", drew)

    g = np.zeros((4, 4), np.int16)
    ck("a patch that fits is applied",
       apply_update(g, 1, 1, 2, 2, [100] * 4) and g[1, 1] == 100)
    before = g.copy()
    ck("a patch running off the edge is refused",
       apply_update(g, 3, 3, 2, 2, [1] * 4) is False
       and np.array_equal(g, before))
    ck("a patch whose data length disagrees is refused",
       apply_update(g, 0, 0, 2, 2, [1, 2, 3]) is False
       and np.array_equal(g, before))

    if HAVE_CV:
        a = np.full((40, 80, 3), 7, np.uint8)
        f = fit(a, 100, 100)
        ck("fit pads to the asked-for size", f.shape == (100, 100, 3))
        ck("fit does not stretch: padding stays black",
           int(f[0, 0].sum()) == 0)

    print()
    if fails:
        print("%d FAILED: %s" % (len(fails), ", ".join(fails)))
        return 1
    print("ALL CHECKS PASSED")
    return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--image-topic", default="/lane/debug_image")
    p.add_argument("--costmap-topic", default="/local_costmap/costmap")
    p.add_argument("--footprint-topic",
                   default="/local_costmap/published_footprint")
    p.add_argument("--out", default=os.path.expanduser("~/esda_runs/combined.mp4"))
    p.add_argument("--seconds", type=float, default=240.0)
    p.add_argument("--fps", type=float, default=10.0)
    p.add_argument("--pane", type=int, default=480, help="pane height, px")
    a = p.parse_args()

    if a.selftest:
        return selftest()
    if not HAVE_CV:
        sys.exit("cv2 missing: sudo apt install -y python3-opencv")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                           QoSReliabilityPolicy, qos_profile_sensor_data)
    from sensor_msgs.msg import Image
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import PolygonStamped
    from map_msgs.msg import OccupancyGridUpdate

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)

    # Nav2 latches the full costmap and then sends only deltas. Default QoS
    # would miss the latched message entirely on a late subscriber.
    latched = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)

    class Rec(Node):
        def __init__(self):
            super().__init__("record_combined")
            self.img = None
            self.grid = None
            self.info = None
            self.foot = None
            self.n = 0
            self.n_img = 0
            self.n_map = 0
            self.n_upd = 0
            self.vw = None
            self.path = None
            self.create_subscription(Image, a.image_topic, self._img,
                                     qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, a.costmap_topic,
                                     self._map, latched)
            self.create_subscription(OccupancyGridUpdate,
                                     a.costmap_topic + "_updates",
                                     self._upd, latched)
            self.create_subscription(PolygonStamped, a.footprint_topic,
                                     self._foot, 10)
            self.create_timer(1.0 / a.fps, self._tick)
            # Say WHY nothing is arriving, while it is still arriving-able.
            # Five recordings came back empty for five different reasons and
            # every one of them needed another seven-minute run to find out.
            # A subscription that matches nothing looks identical to a topic
            # nobody publishes, unless you ask.
            self._checks = [15.0, 45.0]
            self._t0 = time.time()
            self.create_timer(5.0, self._watchdog)

        def _watchdog(self):
            if not self._checks or time.time() - self._t0 < self._checks[0]:
                return
            self._checks.pop(0)
            if self.n_img and self.n_map:
                return
            self.get_logger().warn(
                "frames so far: detector %d, costmap %d, updates %d"
                % (self.n_img, self.n_map, self.n_upd))
            for t in (a.image_topic, a.costmap_topic,
                      a.costmap_topic + "_updates"):
                try:
                    infos = self.get_publishers_info_by_topic(t)
                except Exception as e:                    # noqa: BLE001
                    self.get_logger().warn(f"  {t}: cannot query ({e})")
                    continue
                if not infos:
                    self.get_logger().warn(f"  {t}: NO PUBLISHER")
                    continue
                for i in infos:
                    q = i.qos_profile
                    self.get_logger().warn(
                        "  %s: pub %s  rel=%s dur=%s  (mine: %s)"
                        % (t, i.node_name, q.reliability.name,
                           q.durability.name,
                           "sensor_data/best_effort" if t == a.image_topic
                           else "transient_local/reliable"))
            self.get_logger().info(
                f"{a.image_topic} + {a.costmap_topic} -> {a.out}")

        def _img(self, m):
            if m.encoding not in ("rgb8", "bgr8"):
                return
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            self.img = img[:, :, ::-1] if m.encoding == "rgb8" else img
            self.n_img += 1

        def _map(self, m):
            self.info = m.info
            self.grid = np.asarray(m.data, np.int16).reshape(
                m.info.height, m.info.width)
            self.n_map += 1

        def _upd(self, m):
            if self.grid is None:
                return
            if apply_update(self.grid, m.x, m.y, m.width, m.height, m.data):
                self.n_upd += 1

        def _foot(self, m):
            self.foot = [(pt.x, pt.y) for pt in m.polygon.points]

        def _costmap_pane(self, h, w):
            if self.grid is None:
                pane = np.full((h, w, 3), 30, np.uint8)
                return label(pane, "costmap: waiting")
            img = colourise(self.grid)
            try:
                self._draw_footprint(img)
            except Exception as e:                        # noqa: BLE001
                if not getattr(self, "_foot_warned", False):
                    self._foot_warned = True
                    self.get_logger().warn(
                        f"footprint overlay disabled: {e}")
            out = fit(img, h, w)
            return label(out, "local costmap  %dx%d @ %.2f m  updates %d"
                         % (self.grid.shape[1], self.grid.shape[0],
                            self.info.resolution if self.info else 0.0,
                            self.n_upd))

        def _draw_footprint(self, img):
            """Never let a drawing failure take the recording with it."""
            if self.foot and self.info is not None:
                res = self.info.resolution
                ox, oy = self.info.origin.position.x, self.info.origin.position.y
                H = img.shape[0]
                pts = []
                for x, y in self.foot:
                    px = int((x - ox) / res)
                    py = H - 1 - int((y - oy) / res)     # same flip
                    pts.append([px, py])
                if len(pts) >= 3:
                    cv2.polylines(img, [np.array(pts, np.int32)], True,
                                  (0, 200, 0), 1, cv2.LINE_AA)

        def _tick(self):
            h = a.pane
            w = int(h * 4 / 3)
            left = (fit(self.img, h, w) if self.img is not None
                    else np.full((h, w, 3), 30, np.uint8))
            left = label(left, "lane detector  %s  frames %d"
                         % (a.image_topic, self.n_img))
            right = self._costmap_pane(h, w)
            frame = np.hstack([left, np.full((h, 2, 3), 90, np.uint8), right])
            if self.vw is None:
                self.vw, self.path = open_writer(
                    a.out, frame.shape[1], frame.shape[0], a.fps)
                self.get_logger().info(
                    f"writing {frame.shape[1]}x{frame.shape[0]} -> {self.path}")
            self.vw.write(np.ascontiguousarray(frame))
            self.n += 1

    rclpy.init()
    n = Rec()
    t_end = time.time() + a.seconds
    try:
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if n.vw is not None:
            n.vw.release()
        size = os.path.getsize(n.path) if n.path and os.path.exists(n.path) else 0
        print("[rec] %d frames (%d detector, %d costmap, %d updates) -> %s (%.1f MB)"
              % (n.n, n.n_img, n.n_map, n.n_upd, n.path, size / 1e6))
        n.destroy_node()
        rclpy.shutdown()
    # A file with no costmap data in it is not the recording that was asked
    # for, even if it plays.
    return 0 if (n.n and n.n_img and n.n_map) else 1


if __name__ == "__main__":
    raise SystemExit(main())
