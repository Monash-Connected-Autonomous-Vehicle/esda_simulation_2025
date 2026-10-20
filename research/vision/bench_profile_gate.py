#!/usr/bin/env python3
"""Old vs new lane profile gate, scored on the same frames.

    python3 bench_profile_gate.py --data ~/esda_datasets/bench_20261014_174808 \
        --weights ~/esda_ws/research/models/lane_seg_synth_cpu.pt

WHY NOT eval_recall.py

That scores the MASK. The gate being changed sits two stages downstream,
on Hough segments, so the mask is identical either way and eval_recall
cannot see the difference at all. What has to be scored is the segments
that survive -- because those are what reach the navigator, and a false
one becomes a lethal costmap cell.

WHAT A TRUE POSITIVE IS

A segment is scored against the autolabelled lane polygons, not against
the model's own mask -- scoring a detector against itself proves nothing.
Segments lie on the MASK BOUNDARY (Canny), which is adjacent to the paint
rather than inside it, so the truth is dilated before testing. That
dilation is the one judgement call here, so the result is reported at
three values of it.

A segment counts as true if at least `--hit-frac` of the points sampled
along it fall inside the dilated truth.

The orange-cone gate is skipped for BOTH checks: it is a node method,
unchanged by this work, and leaving it out keeps the comparison to the one
thing that moved.
"""
import argparse
import glob
import importlib.util
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("cv2 missing")


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def old_gate(gray, x1, y1, x2, y2):
    """The base class's verify_line_profile, exactly as it was."""
    mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    dx, dy = float(x2 - x1), float(y2 - y1)
    mag = float(np.hypot(dx, dy))
    if mag == 0.0:
        return False
    nx, ny = -dy / mag, dx / mag
    h, w = gray.shape

    def white(x, y):
        xx, yy = int(x), int(y)
        return bool(0 <= yy < h and 0 <= xx < w and gray[yy, xx] > 120)

    def dark(x, y):
        xx, yy = int(x), int(y)
        if 0 <= yy < h and 0 <= xx < w:
            return bool(gray[yy, xx] < 110)
        return True

    if not white(mx, my):
        return False
    return dark(mx + nx * 12, my + ny * 12) or dark(mx - nx * 12, my - ny * 12)


def seg_is_true(x1, y1, x2, y2, truth_d, n=15, hit_frac=0.6):
    hits = 0
    H, W = truth_d.shape
    for i in range(n):
        t = i / float(n - 1)
        xx, yy = int(x1 + (x2 - x1) * t), int(y1 + (y2 - y1) * t)
        if 0 <= yy < H and 0 <= xx < W and truth_d[yy, xx]:
            hits += 1
    return hits >= hit_frac * n


def selftest():
    fails = []

    def ck(name, cond):
        print("  %-56s %s" % (name, "ok" if cond else "FAIL"))
        if not cond:
            fails.append(name)

    nd_path = os.path.expanduser(
        "~/esda_ws/src/esda_simulation_2025/scripts/lane_detection_esda.py")
    new_gate = load(nd_path, "ndx").edge_profile_ok if os.path.exists(nd_path) else None

    # A real painted line does not have a hard edge: the pixel where the
    # mask boundary falls is a BLEND of paint and ground, which is the
    # measured median of 115. A synthetic hard step does not reproduce the
    # bug at all -- the first version of this test asserted the wrong thing
    # and passed the old gate, which is how the overclaim below was caught.
    g = np.full((200, 200), 40, np.uint8)
    g[:, 95:105] = 220
    g[:, 94] = 115          # the blended edge pixel, as measured
    g[:, 105] = 115
    ck("old gate passes a segment down the middle of the stripe",
       old_gate(g, 100, 60, 100, 140))
    ck("old gate REJECTS a segment on the blended edge -- the bug",
       not old_gate(g, 94, 60, 94, 140))
    if new_gate:
        ck("new gate ACCEPTS that same edge segment -- the fix",
           new_gate(g, 94, 60, 94, 140))

    # CORRECTION. The commit for the new gate claimed it was strictly
    # stricter because the old one would pass a segment inside a wide white
    # blob. It would not: at 12 px out both sides are still white, so the
    # "at least one side dark" clause already rejects it. Both gates reject
    # this. The new gate is differently PLACED, not uniformly stricter.
    blob = np.full((200, 200), 40, np.uint8)
    blob[:, 60:140] = 220
    ck("old gate rejects deep inside a wide blob (not a bug after all)",
       not old_gate(blob, 100, 60, 100, 140))
    if new_gate:
        ck("new gate rejects it too", not new_gate(blob, 100, 60, 100, 140))

    # And where the new gate is genuinely LOOSER: it allows the segment's
    # own pixels to be dark, so any bright-to-dark edge qualifies, paint or
    # not. A shadow boundary passes. That is the whole reason this bench
    # measures precision rather than asserting it.
    shade = np.full((200, 200), 220, np.uint8)
    shade[:, 100:] = 40
    if new_gate:
        ck("new gate accepts a shadow edge -- it cannot tell paint from "
           "any bright/dark boundary", new_gate(shade, 100, 60, 100, 140))

    t = np.zeros((100, 100), bool); t[:, 48:52] = True
    ck("a segment on the truth stripe scores true",
       seg_is_true(50, 10, 50, 90, t))
    ck("a segment far from it scores false",
       seg_is_true(10, 10, 10, 90, t) is False)

    print()
    if fails:
        print("%d FAILED" % len(fails)); return 1
    print("ALL CHECKS PASSED"); return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--data")
    ap.add_argument("--weights")
    ap.add_argument("--vision", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--node", default=os.path.expanduser(
        "~/esda_ws/src/esda_simulation_2025/scripts/lane_detection_esda.py"))
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--hit-frac", type=float, default=0.6)
    ap.add_argument("--mid-sweep", default="0,60,80,95,110,120",
                    help="floor on the segment's own pixel, swept")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not (a.data and a.weights):
        sys.exit("--data and --weights are required")

    er = load(os.path.join(a.vision, "eval_recall.py"), "er")
    nd = load(a.node, "nd")
    from ultralytics import YOLO
    model = YOLO(a.weights)

    imgs = sorted(glob.glob(os.path.join(a.data, "images", "val", "*")))
    if a.limit:
        imgs = imgs[:a.limit]
    print("frames: %d" % len(imgs))

    DILATIONS = (2, 3, 5)
    MIDS = [int(v) for v in a.mid_sweep.split(",") if v != ""]
    keys = ["old", "all"] + ["new%d" % m for m in MIDS]
    stats = {d: {k: [0, 0] for k in keys} for d in DILATIONS}
    n_seg = 0

    for i, ip in enumerate(imgs):
        bgr = cv2.imread(ip)
        if bgr is None:
            continue
        h, w = bgr.shape[:2]
        lp = os.path.join(a.data, "labels", "val",
                          os.path.splitext(os.path.basename(ip))[0] + ".txt")
        truth = er.polys_to_mask(lp, w, h)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        res = model.predict(bgr, verbose=False, conf=0.35)[0]
        raw = er.model_lane_mask(res, w, h, model.names)
        mask = nd.clean_mask((np.asarray(raw) > 0).astype(np.uint8) * 255,
                             nd.roi_mask_lower(h, w),
                             min_area=120, max_area_frac=0.15)
        if not mask.any():
            continue
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=15,
                                minLineLength=30, maxLineGap=40)
        if lines is None:
            continue

        truth_d = {d: cv2.dilate(np.asarray(truth).astype(np.uint8),
                                 np.ones((2 * d + 1, 2 * d + 1), np.uint8)) > 0
                   for d in DILATIONS}

        for ln in lines:
            x1, y1, x2, y2 = ln[0]
            if float(np.hypot(x2 - x1, y2 - y1)) < 30:
                continue
            if abs(float(np.arctan2(y2 - y1, x2 - x1))) < 0.05:
                continue
            n_seg += 1
            o = old_gate(gray, x1, y1, x2, y2)
            nw = {m: nd.edge_profile_ok(gray, x1, y1, x2, y2, mid_min=m)
                  for m in MIDS}
            for d in DILATIONS:
                tru = seg_is_true(x1, y1, x2, y2, truth_d[d], hit_frac=a.hit_frac)
                s = stats[d]
                s["all"][0 if tru else 1] += 1
                if o:
                    s["old"][0 if tru else 1] += 1
                for m in MIDS:
                    if nw[m]:
                        s["new%d" % m][0 if tru else 1] += 1
        if (i + 1) % 50 == 0:
            print("  %d/%d" % (i + 1, len(imgs)), flush=True)

    print("\nsegments after the length and angle gates: %d" % n_seg)
    for d in DILATIONS:
        s = stats[d]
        avail = s["all"][0]
        print("\n--- truth dilated by %d px   (%d of %d segments lie on real paint) ---"
              % (d, avail, sum(s["all"])))
        print("  %-10s %7s %7s %9s %8s" % ("gate", "passed", "true", "precision", "recall"))
        for k in ["old"] + ["new%d" % m for m in MIDS]:
            tp, fp = s[k]
            p = tp / (tp + fp) if tp + fp else float("nan")
            r = tp / avail if avail else float("nan")
            lbl = k if k == "old" else "new mid=%s" % k[3:]
            print("  %-10s %7d %7d %8.3f %8.3f" % (lbl, tp + fp, tp, p, r))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
