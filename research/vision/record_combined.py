#!/usr/bin/env python3
"""record_combined.py -- the detector view beside the costmaps it feeds.

    python3 vision/record_combined.py --out ~/esda_runs/run.mp4 --seconds 240

Three panes on one frame: /lane/debug_image, the Nav2 LOCAL costmap, and the
GLOBAL costmap, each with the robot footprint drawn on it.

Why one file: the question is never what the detector saw or what the
planner believed separately, it is whether the second followed from the
first. Two files with different frame counts and start times cannot answer
that.

Why both costmaps. The LOCAL one is a 10x10 m rolling window with only a
voxel layer and an inflation layer, every source clearing by raytrace, so
it forgets anything not currently in view -- it is a scratchpad, not a
picture of the course. The GLOBAL one is where the course accumulates:
static layer, obstacle layer, a separate lane layer, and the keepout filter
the navigator feeds from its own travel history. Watching only the local
one is what made the costmap look like nothing was happening.

THE COSTMAP IS NOT A PLAIN TOPIC. Nav2 publishes the full OccupancyGrid
latched (transient-local) and may send only deltas on <topic>_updates. A
default-QoS subscription gets nothing, or one frozen frame -- which records
perfectly happily. So the QoS is matched and the patches applied.

Colours:
    dark grey   unknown (-1)
    light grey  free (0)        -- not white: the inflation ramp has to show
    blue->red   rising cost (1-98)
    cyan        inscribed (99)
    magenta     lethal (100)
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

UNKNOWN = (55, 55, 55)
FREE = (225, 225, 225)         # grey, not white: low cost must be visible
INSCRIBED = (255, 255, 0)      # BGR cyan
LETHAL = (255, 0, 255)         # BGR magenta


def cost_lut():
    """256-entry BGR lookup indexed by cost+1, so -1 lands at 0.

    The ramp starts well away from FREE. With free at white and cost 1 a
    near-white blue, a 12-cell inflation skirt around every lane pixel read
    as a faint smudge on a white field -- which is exactly how the first
    recording looked.
    """
    lut = np.zeros((256, 3), np.uint8)
    lut[0] = UNKNOWN
    lut[1] = FREE
    for v in range(1, 99):
        t = (v - 1) / 97.0
        # blue -> cyan-ish -> orange -> red, starting at a clearly non-grey blue
        b = int(235 * (1 - t) ** 0.7)
        g = int(170 * (1 - abs(t - 0.45) / 0.55)) if 0 <= t <= 1 else 0
        r = int(40 + 215 * t ** 0.8)
        lut[v + 1] = (max(0, b), max(0, g), min(255, r))
    lut[100] = INSCRIBED
    lut[101] = LETHAL
    return lut


LUT = cost_lut()


def colourise(grid):
    """int8 grid HxW -> BGR, origin row at the BOTTOM of the image.

    ascontiguousarray, not just flipud: flipud returns a negative-stride
    view and cv2 drawing refuses it, mid-recording rather than at startup.
    """
    g = np.asarray(grid, dtype=np.int16)
    idx = np.clip(g + 1, 0, 101).astype(np.uint8)
    return np.ascontiguousarray(np.flipud(LUT[idx]))


def apply_update(grid, x, y, w, h, data):
    """Patch an OccupancyGridUpdate in place. False if it does not fit."""
    H, W = grid.shape
    if x < 0 or y < 0 or x + w > W or y + h > H or len(data) != w * h:
        return False
    grid[y:y + h, x:x + w] = np.asarray(data, dtype=np.int16).reshape(h, w)
    return True


def label(img, text, scale=0.5):
    cv2.rectangle(img, (0, 0), (img.shape[1], 24), (0, 0, 0), -1)
    cv2.putText(img, text, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, scale,
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


def footprint_topic_for(costmap_topic):
    """/local_costmap/costmap -> /local_costmap/published_footprint"""
    return costmap_topic.rsplit("/", 1)[0] + "/published_footprint"


def selftest():
    fails = []

    def ck(name, cond):
        print("  %-58s %s" % (name, "ok" if cond else "FAIL"))
        if not cond:
            fails.append(name)

    ck("unknown is dark grey", tuple(int(v) for v in LUT[0]) == UNKNOWN)
    ck("free is grey, not white", tuple(int(v) for v in LUT[1]) == FREE)
    ck("cost 1 is clearly not free", abs(int(LUT[2][0]) - FREE[0]) > 40
       or abs(int(LUT[2][2]) - FREE[2]) > 40)
    ck("inscribed differs from lethal", tuple(LUT[100]) != tuple(LUT[101]))
    ck("red rises monotonically with cost",
       all(int(LUT[v + 1][2]) <= int(LUT[v + 2][2]) for v in range(1, 97)))

    g = np.array([[0, 0], [100, -1]], np.int16)
    img = colourise(g)
    ck("colourise keeps the shape", img.shape == (2, 2, 3))
    ck("grid row 0 lands at the BOTTOM of the image",
       tuple(int(v) for v in img[1, 0]) == FREE)
    ck("lethal survives the flip", tuple(int(v) for v in img[0, 0]) == LETHAL)
    ck("the flipped image is contiguous", img.flags["C_CONTIGUOUS"])
    if HAVE_CV:
        try:
            cv2.polylines(img, [np.array([[0, 0], [1, 0], [1, 1]], np.int32)],
                          True, (0, 200, 0), 1)
            drew = True
        except Exception:
            drew = False
        ck("cv2 will draw on a colourise() result", drew)

    g = np.zeros((4, 4), np.int16)
    ck("a patch that fits is applied",
       apply_update(g, 1, 1, 2, 2, [100] * 4) and g[1, 1] == 100)
    before = g.copy()
    ck("a patch off the edge is refused",
       apply_update(g, 3, 3, 2, 2, [1] * 4) is False and np.array_equal(g, before))
    ck("a patch of the wrong length is refused",
       apply_update(g, 0, 0, 2, 2, [1, 2, 3]) is False and np.array_equal(g, before))

    ck("footprint topic is derived from the costmap topic",
       footprint_topic_for("/local_costmap/costmap")
       == "/local_costmap/published_footprint")
    ck("and for the global one",
       footprint_topic_for("/global_costmap/costmap")
       == "/global_costmap/published_footprint")

    if HAVE_CV:
        a = np.full((40, 80, 3), 7, np.uint8)
        f = fit(a, 100, 100)
        ck("fit pads to the asked-for size", f.shape == (100, 100, 3))
        ck("fit pads rather than stretching", int(f[0, 0].sum()) == 0)

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
    p.add_argument("--costmap-topics", nargs="*",
                   default=["/local_costmap/costmap", "/global_costmap/costmap"])
    p.add_argument("--out", default=os.path.expanduser("~/esda_runs/combined.mp4"))
    p.add_argument("--seconds", type=float, default=240.0)
    p.add_argument("--fps", type=float, default=10.0)
    p.add_argument("--pane", type=int, default=520, help="pane height, px")
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
    latched = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)

    class Rec(Node):
        def __init__(self):
            super().__init__("record_combined")
            self.img = None
            self.n_img = 0
            self.n = 0
            self.vw = None
            self.path = None
            self.maps = {t: {"grid": None, "info": None, "foot": None,
                             "n": 0, "upd": 0} for t in a.costmap_topics}
            self.create_subscription(Image, a.image_topic, self._img,
                                     qos_profile_sensor_data)
            for t in a.costmap_topics:
                self.create_subscription(
                    OccupancyGrid, t,
                    (lambda tt: lambda m: self._map(tt, m))(t), latched)
                self.create_subscription(
                    OccupancyGridUpdate, t + "_updates",
                    (lambda tt: lambda m: self._upd(tt, m))(t), latched)
                self.create_subscription(
                    PolygonStamped, footprint_topic_for(t),
                    (lambda tt: lambda m: self._foot(tt, m))(t), 10)
            self.create_timer(1.0 / a.fps, self._tick)
            self._checks = [15.0, 45.0]
            self._t0 = time.time()
            self.create_timer(5.0, self._watchdog)
            self.get_logger().info(
                "%s + %s -> %s" % (a.image_topic, ", ".join(a.costmap_topics),
                                   a.out))

        # ---- inputs -------------------------------------------------------
        def _img(self, m):
            if m.encoding not in ("rgb8", "bgr8"):
                return
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            self.img = img[:, :, ::-1] if m.encoding == "rgb8" else img
            self.n_img += 1

        def _map(self, t, m):
            s = self.maps[t]
            s["info"] = m.info
            s["grid"] = np.asarray(m.data, np.int16).reshape(
                m.info.height, m.info.width)
            s["n"] += 1

        def _upd(self, t, m):
            s = self.maps[t]
            if s["grid"] is None:
                return
            if apply_update(s["grid"], m.x, m.y, m.width, m.height, m.data):
                s["upd"] += 1

        def _foot(self, t, m):
            self.maps[t]["foot"] = [(pt.x, pt.y) for pt in m.polygon.points]

        # ---- diagnosis ----------------------------------------------------
        def _watchdog(self):
            if not self._checks or time.time() - self._t0 < self._checks[0]:
                return
            self._checks.pop(0)
            if self.n_img and all(s["n"] for s in self.maps.values()):
                return
            self.get_logger().warn("detector frames %d; %s" % (
                self.n_img, "; ".join("%s %d" % (t.split("/")[1], s["n"])
                                      for t, s in self.maps.items())))
            for t in [a.image_topic] + list(self.maps):
                try:
                    infos = self.get_publishers_info_by_topic(t)
                except Exception as e:                    # noqa: BLE001
                    self.get_logger().warn(f"  {t}: cannot query ({e})")
                    continue
                if not infos:
                    self.get_logger().warn(f"  {t}: NO PUBLISHER")
                for i in infos:
                    q = i.qos_profile
                    self.get_logger().warn(
                        "  %s: %s rel=%s dur=%s" % (t, i.node_name,
                                                    q.reliability.name,
                                                    q.durability.name))

        # ---- drawing ------------------------------------------------------
        def _draw_footprint(self, img, s):
            if not s["foot"] or s["info"] is None:
                return
            res = s["info"].resolution
            ox = s["info"].origin.position.x
            oy = s["info"].origin.position.y
            H = img.shape[0]
            pts = [[int((x - ox) / res), H - 1 - int((y - oy) / res)]
                   for x, y in s["foot"]]
            if len(pts) >= 3:
                cv2.polylines(img, [np.array(pts, np.int32)], True,
                              (0, 190, 0), 2, cv2.LINE_AA)

        def _pane(self, t, h, w):
            s = self.maps[t]
            name = t.split("/")[1].replace("_costmap", "")
            if s["grid"] is None:
                return label(np.full((h, w, 3), 28, np.uint8),
                             "%s costmap: waiting" % name)
            img = colourise(s["grid"])
            try:
                self._draw_footprint(img, s)
            except Exception as e:                        # noqa: BLE001
                if not getattr(self, "_fw", False):
                    self._fw = True
                    self.get_logger().warn(f"footprint overlay off: {e}")
            gh, gw = s["grid"].shape
            res = s["info"].resolution if s["info"] else 0.0
            return label(fit(img, h, w),
                         "%s costmap  %dx%d cells  %.0fx%.0f m  msgs %d"
                         % (name, gw, gh, gw * res, gh * res, s["n"]))

        def _tick(self):
            h = a.pane
            dw = int(h * 4 / 3)
            left = (fit(self.img, h, dw) if self.img is not None
                    else np.full((h, dw, 3), 28, np.uint8))
            left = label(left, "lane detector  frames %d" % self.n_img)
            panes = [left]
            for t in a.costmap_topics:
                panes.append(np.full((h, 2, 3), 90, np.uint8))
                panes.append(self._pane(t, h, h))
            frame = np.hstack(panes)
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
        print("[rec] %d frames, detector %d, %s -> %s (%.1f MB)"
              % (n.n, n.n_img,
                 ", ".join("%s %d(+%d upd)" % (t.split("/")[1], s["n"], s["upd"])
                           for t, s in n.maps.items()),
                 n.path, size / 1e6))
        ok = bool(n.n and n.n_img and any(s["n"] for s in n.maps.values()))
        n.destroy_node()
        rclpy.shutdown()
    # A pane that never received anything is not the recording that was asked
    # for, even though the file plays.
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
