#!/usr/bin/env python3
"""lane_classic.py -- find the painted lane without a neural network.

WHY THIS EXISTS. The IGVC AutoNav 2025 first-place run's own debug view is a
binary image: white lines on black inside a trapezoid. No segmentation network
in sight. Painted lines are the easiest thing in the scene -- bright, thin,
high contrast against asphalt or grass -- and a threshold finds them in well
under a millisecond, against roughly 8 ms for YOLOv8n-seg on a 5060 Ti and
considerably more on a Jetson.

Our YOLO weights currently score 0.174 recall at the camera the robot actually
carries. Before spending an hour of GPU on a retrain it is worth knowing what
a threshold scores on the same frames, because if the answer is "much better"
then the perception problem was never a model problem.

FOUR METHODS, because which one wins is an empirical question:

  otsu      global Otsu on greyscale, keep the bright class. Simplest. Fails
            when sun and shade split the frame, since one global level cannot
            serve both.
  value     bright AND desaturated in HSV. Paint is white; grass and asphalt
            are not. Uses colour, so it is the one that would suffer most from
            feeding the pipeline greyscale.
  adaptive  threshold against a local mean. Survives a brightness gradient
            across the frame.
  tophat    white top-hat: subtract a morphological opening, which deletes
            anything wider than the structuring element and leaves thin bright
            ridges. A lane line IS a thin bright ridge, and the result is
            invariant to the background level, so shade and sun threshold the
            same. This is the one to beat.

Every method ends the same way: keep only pixels below the horizon (derived
from the camera geometry, not hardcoded), drop connected components too small
to be a line, and close small gaps.

    python3 vision/lane_classic.py --selftest
    python3 vision/eval_recall.py --data <ds> --detector classic --method tophat
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import cv2
    HAVE_CV = True
except ImportError:
    HAVE_CV = False

METHODS = ("otsu", "value", "adaptive", "tophat")
_HORIZON = {}
_ROI = {}


def horizon_row(pitch_deg=0.0, max_range=6.0):
    """First image row inside the ROI, from the camera geometry itself."""
    key = (pitch_deg, max_range)
    if key not in _HORIZON:
        rows = np.nonzero(ground_roi(pitch_deg, max_range).any(axis=1))[0]
        _HORIZON[key] = int(rows[0]) if len(rows) else 0
    return _HORIZON[key]


def ground_roi(pitch_deg=0.0, max_range=6.0):
    """Per-pixel mask of where the ground actually is, from the projection.

    A single horizon ROW is not enough. Above the horizon there is sky, but
    past the horizon row there is also the far edge of the arena and the grey
    band beyond it, and all of it is brighter than grass -- the first run of
    this detector happily reported the skyline as a lane. pixel_to_ground
    already marks a pixel invalid if it is above the horizon OR beyond
    MAX_X, which is exactly the region a lane cannot be in, so use that
    rather than a row number. It is the same projection the labels and the
    lane node use, so the detector is confined to the area it is scored on.
    """
    key = (pitch_deg, max_range)
    if key not in _ROI:
        import synth_gen as sg
        X, _, valid = sg.pixel_to_ground(pitch_deg)
        # synth_gen's MAX_X is 30 m, so "valid" reaches the far side of the
        # arena: rows 137-200 are ground between 4.6 and 29 m, which is where
        # the arena's bright edge and the grey border live. The stack consumes
        # lane points to 4 m and the scoring bins stop at 6, so paint at 25 m
        # is neither usable nor scored -- and reporting it is pure noise.
        _ROI[key] = valid & (X <= max_range)
    return _ROI[key]


def _clean(mask, min_area, close_px):
    if close_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    if min_area > 0:
        n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        keep = np.zeros(n, bool)
        for i in range(1, n):
            keep[i] = stats[i, cv2.CC_STAT_AREA] >= min_area
        mask = (keep[lab]).astype(np.uint8)
    return mask


def lane_mask(bgr, method="tophat", pitch_deg=0.0, min_area=120, close_px=3,
              tophat_px=25, tophat_thresh=12, adaptive_block=61, adaptive_c=-8,
              v_min=170, s_max=60, roi_top=None, max_range=6.0,
              max_area_frac=0.15):
    """Binary lane mask, same shape as the image. True where paint is.

    max_area_frac is a sanity check, and it is not optional.

    PAINT IS THIN. Measured on the teleport bench, a real detection covers
    0.7% to 3.5% of the ROI. A mask covering a sixth of the drivable ground
    is not a lane, whatever the threshold thinks.

    This exists because otsu has no way to decline. It always splits the
    histogram into two classes, even when the image contains only one thing,
    so a frame of bare asphalt with no paint in it gets split into "darker
    asphalt" and "lighter asphalt" and the lighter half is reported as lane.
    The self-test below has said so since the day it was written -- otsu is
    excluded from the blank-field check with a note that a global threshold
    needs a guard in front of it -- and the guard was never added. On the
    IGVC course the robot drove into open ground behind a wall of lane cells
    that were a flat grey surface being thresholded against itself.

    It matters more here than it would elsewhere because the lane is written
    into the costmap as a LETHAL obstacle. A missed line costs a detection; a
    false line builds a wall across drivable ground. The failures are not
    symmetric, so the guard is biased to refuse.
    """
    if method not in METHODS:
        raise ValueError(f"method must be one of {METHODS}")
    h, w = bgr.shape[:2]
    grey = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    if method == "otsu":
        _, m = cv2.threshold(grey, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    elif method == "value":
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m = ((hsv[..., 2] >= v_min) & (hsv[..., 1] <= s_max)).astype(np.uint8)
    elif method == "adaptive":
        blk = adaptive_block | 1
        m = cv2.adaptiveThreshold(grey, 1, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv2.THRESH_BINARY, blk, adaptive_c)
    else:  # tophat
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (tophat_px, tophat_px))
        th = cv2.morphologyEx(grey, cv2.MORPH_TOPHAT, k)
        m = (th >= tophat_thresh).astype(np.uint8)

    if roi_top is None:
        roi = ground_roi(pitch_deg, max_range)
        if roi.shape != (h, w):
            roi = cv2.resize(roi.astype(np.uint8), (w, h),
                             interpolation=cv2.INTER_NEAREST).astype(bool)
        m[~roi] = 0
        roi_px = int(roi.sum())
    elif roi_top > 0:
        m[:min(roi_top, h)] = 0
        roi_px = (h - min(roi_top, h)) * w
    else:
        roi_px = h * w
    out = _clean(m, min_area, close_px).astype(bool)
    if max_area_frac > 0.0 and roi_px > 0 and out.sum() > max_area_frac * roi_px:
        return np.zeros_like(out)
    return out


# ----------------------------------------------------------------- selftest --
def selftest():
    print("=" * 72)
    print("lane_classic self-test -- synthetic images, known answers")
    print("=" * 72)
    fails = []

    def ck(name, cond):
        print("  %-56s %s" % (name, "ok" if cond else "FAIL"))
        if not cond:
            fails.append(name)

    # Sized from the camera, not written down: the ROI is the camera's own
    # ground mask, so a test image of a different shape compares two
    # different-sized arrays and fails for no real reason.
    import synth_gen as sg
    H, W = sg.H_IMG, sg.W
    TOP = int(H * 0.55)             # a fixed ROI top for the method tests

    LX0, LX1 = W // 2 - 4, W // 2 + 5   # the line's own columns, see scene()

    def scene(bg, line, width=9, x=None):
        x = W // 2 if x is None else x
        img = np.full((H, W, 3), bg, np.uint8)
        img[:, x - width // 2:x + width // 2 + 1] = line
        return img

    # 1. a bright line on a dark field: every method should find it
    img = scene(60, 220)
    for mth in METHODS:
        m = lane_mask(img, mth, roi_top=TOP)
        hit = m[TOP:, LX0:LX1].mean()
        off = m[TOP:, :250].mean()
        ck(f"{mth}: finds a bright line on a dark field", hit > 0.5)
        ck(f"{mth}: does not paint the empty field", off < 0.02)

    # 2. a blank field: nothing should be detected. Otsu is expected to fail
    #    here -- it always splits into two classes, even when there is only
    #    one thing in the image. That is a real property, not a bug, and it is
    #    why a global threshold needs a sanity check in front of it.
    blank = np.full((H, W, 3), 120, np.uint8)
    for mth in ("value", "adaptive", "tophat"):
        ck(f"{mth}: blank field gives (almost) nothing",
           lane_mask(blank, mth, roi_top=TOP).mean() < 0.02)

    # 3. THE ONE THAT MATTERS: the same line with the background lifted, as
    #    sun or a pale surface would. A method that only works at one exposure
    #    is no use on a course that has shade on it.
    for bg, line in ((40, 200), (120, 255), (150, 235)):
        img = scene(bg, line)
        for mth in ("tophat", "adaptive"):
            m = lane_mask(img, mth, roi_top=TOP)
            ck(f"{mth}: still finds the line at background {bg}",
               m[TOP:, LX0:LX1].mean() > 0.8)

    # 4. low contrast, which is what our SIM renders: pale line on green turf.
    low = np.full((H, W, 3), (70, 110, 70), np.uint8)
    low[:, W // 2 - 5:W // 2 + 6] = (120, 165, 120)
    for mth in ("tophat", "adaptive"):
        ck(f"{mth}: finds a LOW-contrast line (as the sim renders it)",
           lane_mask(low, mth, roi_top=TOP)[TOP:, LX0:LX1].mean() > 0.8)

    # 5. the ROI really is applied, and small specks really are dropped
    img = scene(60, 220)
    ck("nothing is reported above the ROI top",
       not lane_mask(img, "tophat", roi_top=TOP)[:TOP].any())
    speck = np.full((H, W, 3), 60, np.uint8)
    speck[TOP + 20:TOP + 24, W // 2:W // 2 + 4] = 230
    ck("a 4x4 speck is dropped by min_area",
       not lane_mask(speck, "tophat", roi_top=TOP, min_area=120).any())
    ck("...and kept when min_area allows it",
       lane_mask(speck, "tophat", roi_top=TOP, min_area=4, close_px=0).any())

    # 6. the ROI comes from the camera projection, not from a magic number,
    #    and it must exclude the sky -- the first version cut by row only and
    #    reported the skyline as a lane.
    hr = horizon_row(0.0)
    roi = ground_roi(0.0)
    ck(f"horizon row {hr} is inside the image", 0 < hr < H)
    ck("the ROI is a per-pixel mask the size of the image", roi.shape == (H, W))
    ck("nothing above the horizon is in the ROI", not roi[:hr].any())
    # On a flat ground plane with no roll the range depends only on the row,
    # so the ROI is "rows below N" and IS everything below its own horizon.
    # The property that matters is that a tighter range starts LOWER down.
    ck("the ROI covers everything below its own horizon", roi[hr:].all())
    ck("a 6 m ROI starts lower in the frame than a 30 m one",
       horizon_row(0.0, 6.0) > horizon_row(0.0, 30.0))
    ck("a 4 m ROI starts lower still",
       horizon_row(0.0, 4.0) > horizon_row(0.0, 6.0))
    ck("the ROI is a strict subset of the frame", roi.mean() < 0.95)
    bright_sky = np.full((H, W, 3), 60, np.uint8)
    bright_sky[:hr + 10] = 235
    ck("a bright sky is not reported as lane",
       lane_mask(bright_sky, "tophat").mean() < 0.01)

    # 7. THE GUARD. A frame of bare ground with no paint must give nothing,
    #    including from otsu, which cannot decline on its own.
    bare = np.random.default_rng(0).normal(120, 6, (H, W, 3)).clip(0, 255).astype(np.uint8)
    for mth in METHODS:
        got = lane_mask(bare, mth).mean()
        ck(f"{mth}: bare ground with no paint gives nothing", got < 0.001)
    ck("otsu WITHOUT the guard would have flooded the frame",
       lane_mask(bare, "otsu", max_area_frac=0.0).mean() > 0.2)

    #    ...and a real line must still survive the guard.
    real = np.full((H, W, 3), 70, np.uint8)
    real[:, W // 2 - 4:W // 2 + 5] = 230
    for mth in ("otsu", "adaptive", "tophat"):
        ck(f"{mth}: a real thin line still passes the guard",
           lane_mask(real, mth)[horizon_row():, LX0:LX1].mean() > 0.8)
    ck("the guard rejects a mask over the fraction",
       not lane_mask(real, "otsu", max_area_frac=0.0001).any())

    print()
    if fails:
        print(f"{len(fails)} FAILED: " + ", ".join(fails))
        raise SystemExit(1)
    print("ALL CHECKS PASSED")
    print("=" * 72)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--image", help="run one image and write <image>_mask.png")
    p.add_argument("--method", default="tophat", choices=METHODS)
    a = p.parse_args()
    if not HAVE_CV:
        raise SystemExit("needs opencv")
    if a.selftest:
        return selftest()
    if not a.image:
        raise SystemExit("--image or --selftest")
    img = cv2.imread(a.image, cv2.IMREAD_COLOR)
    m = lane_mask(img, a.method)
    out = os.path.splitext(a.image)[0] + f"_{a.method}.png"
    cv2.imwrite(out, (m * 255).astype(np.uint8))
    print(f"{m.sum()} lane pixels -> {out}")


if __name__ == "__main__":
    raise SystemExit(main())
