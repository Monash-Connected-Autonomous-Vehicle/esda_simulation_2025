#!/usr/bin/env python3
"""backtest_detectors.py -- theirs against ours, same frames, same scoring.

THE QUESTION. esda_simulation_2025's lane_detection.py thresholds grayscale at
a fixed level (white_threshold_low, default 130). lane_detection_esda.py keeps
every other stage of that node identical -- the ROI, Canny, Hough,
filter_valid_lanes, the depth projection -- and changes only how the mask is
produced. So the honest comparison is between the MASKS, with everything
downstream held constant. That is what this measures.

WHY A MASK COMPARISON IS THE RIGHT ONE. It is tempting to compare the final
line segments instead, but both pipelines run the same Hough and the same
dark-white-dark filter on whatever mask they are given. Any difference in the
lines is therefore caused entirely by the mask, and measuring the mask
measures it without the extra variance Hough adds.

THE FRAMES. The teleport bench: the camera placed at planned poses with the
pose COMMANDED rather than integrated from odometry, so the ground truth comes
from the world's own geometry and cannot drift. An earlier driven capture had
labels half a metre out by the five-minute mark and a third of its frames
duplicated from a wedged robot; none of that can happen here.

THE LIGHTING. Each method is also scored under four perturbations applied to
the image only, leaving geometry and labels untouched: a brightness shift both
ways, a gradient across the frame, and a hard shadow band. A sim lights every
frame identically, which is the one condition a fixed global threshold is
built for and an outdoor course never provides.

    python3 vision/backtest_detectors.py --data ~/esda_datasets/bench_XXXX
    python3 vision/backtest_detectors.py --selftest
"""
import argparse
import glob
import importlib.util
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

DEFAULT_ESDA = ("/mnt/c/Users/felix/Downloads/esda_simulation_2025-esda-jetson-testing/"
                "esda_simulation_2025-esda-jetson-testing/src/esda_simulation_2025/"
                "scripts/lane_detection_esda.py")

# theirs, then ours, then the two we are not proposing but measured anyway
METHODS = [("fixed", "theirs (lane_detection.py, grayscale > 130)"),
           ("adaptive", "OURS (local mean)"),
           ("otsu", "per-frame global level"),
           ("tophat", "morphological ridge")]
PERTURBS = ["none", "bright", "dark", "gradient", "shadow"]


def load_esda(path):
    """Import lane_detection_esda.py by path.

    Its rclpy and lane_detection imports live inside _build_node_class(), so
    importing the module does not need ROS -- which is the whole reason the
    mask functions were put at module level.
    """
    if not os.path.exists(path):
        raise SystemExit(
            f"cannot find lane_detection_esda.py at:\n  {path}\n"
            f"pass --esda <path to it>")
    spec = importlib.util.spec_from_file_location("lane_detection_esda", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def score_one(esda, eval_recall, cv2, imgs, data, split, method, perturb, bins, edges):
    tot = (0, 0, 0)
    per = [(0, 0, 0)] * len(bins)
    ms, fired, with_truth, found = [], 0, 0, 0
    for ip in imgs:
        img = cv2.imread(ip)
        if perturb != "none":
            img = eval_recall.perturb_image(img, perturb)
        h, w = img.shape[:2]
        lp = os.path.join(data, "labels", split,
                          os.path.splitext(os.path.basename(ip))[0] + ".txt")
        truth = eval_recall.polys_to_mask(lp, w, h)

        t0 = time.monotonic()
        m = esda.raw_mask(img, method)
        pred = esda.clean_mask(m, esda.roi_mask_lower(h, w)) > 0
        ms.append((time.monotonic() - t0) * 1e3)

        if pred.any():
            fired += 1
        if truth.any():
            with_truth += 1
            if (pred & truth).any():
                found += 1
        t, p_ = eval_recall.accumulate(pred, truth, bins)
        tot = eval_recall.add(tot, t)
        per = [eval_recall.add(a, b) for a, b in zip(per, p_)]

    pr, rc, iou = eval_recall.prf(*tot)
    band = None
    for (lo, hi), c in zip(zip(edges[:-1], edges[1:]), per):
        if (lo, hi) == (1.0, 2.0):
            band = eval_recall.prf(*c)[1]
    return dict(recall=rc, precision=pr, iou=iou, band=band,
                ms=float(np.median(ms)), fired=fired,
                frame_hit=(found / with_truth if with_truth else float("nan")))


def selftest():
    """Synthetic frames with known answers -- no bench dataset needed."""
    import cv2
    print("=" * 70)
    print("backtest harness self-test")
    print("=" * 70)
    fails = []

    def ck(name, cond):
        print("  %-56s %s" % (name, "ok" if cond else "FAIL"))
        if not cond:
            fails.append(name)

    esda = load_esda(os.environ.get("ESDA_SCRIPT", DEFAULT_ESDA))
    ck("lane_detection_esda imports without ROS", hasattr(esda, "raw_mask"))
    ck("it exposes the methods we score", set(m for m, _ in METHODS) <= set(esda.METHODS))

    import eval_recall
    ck("eval_recall exposes the scoring maths",
       all(hasattr(eval_recall, a) for a in
           ("polys_to_mask", "accumulate", "prf", "add", "range_bins",
            "perturb_image")))

    H, W = 480, 640
    img = np.full((H, W, 3), 60, np.uint8)
    img[:, W // 2 - 4:W // 2 + 5] = 230
    roi = esda.roi_mask_lower(H, W)
    for mth, _ in METHODS:
        m = esda.clean_mask(esda.raw_mask(img, mth), roi) > 0
        ck(f"{mth}: finds a synthetic line", m[H // 2:, W // 2 - 4:W // 2 + 5].mean() > 0.8)

    # The discriminating case: ground brighter than the fixed level.
    bright = np.full((H, W, 3), 150, np.uint8)
    bright[:, W // 2 - 4:W // 2 + 5] = 235
    fx = esda.clean_mask(esda.raw_mask(bright, "fixed"), roi) > 0
    ad = esda.clean_mask(esda.raw_mask(bright, "adaptive"), roi) > 0
    ck("fixed fails when the ground outshines its threshold", not fx.any())
    ck("adaptive still finds it", ad[H // 2:, W // 2 - 4:W // 2 + 5].mean() > 0.8)

    print()
    if fails:
        print(f"{len(fails)} FAILED: " + ", ".join(fails))
        return 1
    print("ALL CHECKS PASSED")
    print("=" * 70)
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--data", help="bench dataset from autolabel.py")
    p.add_argument("--split", default="val")
    p.add_argument("--esda", default=DEFAULT_ESDA)
    p.add_argument("--limit", type=int, default=0)
    p.add_argument("--edges", default="0,1,2,3,4,6")
    a = p.parse_args()

    if a.selftest:
        return selftest()
    if not a.data:
        raise SystemExit("--data is required (or --selftest)")

    import cv2
    import eval_recall
    import synth_gen as sg

    esda = load_esda(a.esda)
    data = os.path.expanduser(a.data)
    imgs = sorted(glob.glob(os.path.join(data, "images", a.split, "*.jpg")))
    if a.limit:
        imgs = imgs[:a.limit]
    if not imgs:
        raise SystemExit(f"no images under {data}/images/{a.split}")

    # Score each DISTINCT frame once. A wedged robot emits byte-identical
    # frames and they would otherwise weight the result toward one view.
    import hashlib
    seen, keep = set(), []
    for ip in imgs:
        d = hashlib.md5(open(ip, "rb").read()).hexdigest()
        if d not in seen:
            seen.add(d)
            keep.append(ip)
    print(f"{len(keep)} distinct frames of {len(imgs)}  ({data})")
    print(f"detector source: {a.esda}\n")
    imgs = keep

    edges = [float(e) for e in a.edges.split(",")]
    X, _Y, valid = sg.pixel_to_ground()
    bins = eval_recall.range_bins(X, valid, edges)

    results = {}
    for mth, label in METHODS:
        for pb in PERTURBS:
            results[(mth, pb)] = score_one(esda, eval_recall, cv2, imgs, data,
                                           a.split, mth, pb, bins, edges)
            print(".", end="", flush=True)
    print("\n")

    for title, key in (("RECALL", "recall"), ("PRECISION", "precision"),
                       ("IoU", "iou"), ("RECALL at 1-2 m (the carrot band)", "band")):
        print(f"=== {title} ===")
        print("  %-10s %8s %8s %8s %8s %8s %9s" %
              ("method", *PERTURBS, "WORST"))
        for mth, _ in METHODS:
            vals = [results[(mth, pb)][key] for pb in PERTURBS]
            vals = [v if v is not None and v == v else float("nan") for v in vals]
            print("  %-10s %s %9.3f" %
                  (mth, " ".join("%8.3f" % v for v in vals), min(vals)))
        print()

    print("=== cost, unperturbed ===")
    for mth, label in METHODS:
        r = results[(mth, "none")]
        print("  %-10s %6.2f ms/frame   fired in %d/%d frames   %-s"
              % (mth, r["ms"], r["fired"], len(imgs), label))

    print()
    print("READ: the WORST column, not the clean one. The course picks the")
    print("lighting and the robot does not get to retry. A method that starts")
    print("high and collapses under a shadow is worse than one that never")
    print("moves, however good its best frame looks.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
