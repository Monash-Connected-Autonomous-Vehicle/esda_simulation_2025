#!/usr/bin/env python3
"""Pixel recall for the lane class, BY RANGE. The number we have never had.

WHY PIXELS AND NOT BOXES

Ultralytics reports box and mask mAP, which is the right metric for a
detector. It is not the right metric for this robot. What the stack consumes
is the lane MASK, projected to ground points, fitted to a centreline. A
detection with a perfect box and half the paint missing steers badly; a
ragged box around all the paint steers fine. So: per-pixel TP/FP/FN against
the ground truth autolabel.py computes from the world's own geometry.

WHY BY RANGE

This is the whole point. Lane paint at 1 m is dozens of pixels across; at
4 m it is a few. A single recall figure averages those together and hides
the only thing that matters operationally -- how far ahead the robot can
actually see the line. The driver puts its carrot 1.75-2.75 m out, so recall
in that band is what decides whether it drives.

Every pixel's ground range is fixed by the camera geometry, so it is
computed once for the whole dataset rather than per frame.

    python3 eval_recall.py --data ~/esda_datasets/eval --weights ~/esda_models/lane_seg.pt
    python3 eval_recall.py --selftest        # no model, no dataset needed
"""
import argparse
import glob
import os
import sys

import numpy as np

LANE_CLASS = 0


# --------------------------------------------------------------------------
# scoring -- pure, no model, no I/O, so it can be tested offline
# --------------------------------------------------------------------------
def confusion(pred, truth):
    """TP, FP, FN pixel counts for two boolean masks."""
    pred = pred.astype(bool)
    truth = truth.astype(bool)
    tp = int(np.count_nonzero(pred & truth))
    fp = int(np.count_nonzero(pred & ~truth))
    fn = int(np.count_nonzero(~pred & truth))
    return tp, fp, fn


def prf(tp, fp, fn):
    """Precision, recall, IoU. Undefined cases return nan, not a fake zero."""
    p = tp / (tp + fp) if (tp + fp) else float("nan")
    r = tp / (tp + fn) if (tp + fn) else float("nan")
    i = tp / (tp + fp + fn) if (tp + fp + fn) else float("nan")
    return p, r, i


def range_bins(x_of_pixel, valid, edges):
    """Boolean mask per range bin, from the per-pixel ground distance."""
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        out.append(valid & (x_of_pixel >= lo) & (x_of_pixel < hi))
    return out


def accumulate(pred, truth, bins):
    """TP/FP/FN overall and within each range bin."""
    tot = confusion(pred, truth)
    per = [confusion(pred & b, truth & b) for b in bins]
    return tot, per


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


# --------------------------------------------------------------------------
PERTURBS = ("none", "bright", "dark", "gradient", "shadow")


def perturb_image(bgr, kind, strength=50):
    """Change the LIGHTING without changing the geometry.

    The sim lights every frame uniformly, so the bench cannot tell a detector
    that survives a shadow edge from one that does not -- and that is the one
    property separating a global threshold from a local one. Rather than
    re-render the world, bend the light on the frames we already have:

      bright/dark  a global shift, i.e. exposure or overcast-versus-sun.
      gradient     a linear ramp across the frame: low sun from one side.
      shadow       a hard dark band over part of the image: a tree edge. This
                   is the one that kills a global threshold, because no single
                   level serves both halves.

    Geometry is untouched, so the SAME ground truth still applies and the
    numbers stay comparable to the unperturbed run.
    """
    import numpy as _np
    if kind in (None, "none"):
        return bgr
    h, w = bgr.shape[:2]
    img = bgr.astype(_np.int16)
    if kind == "bright":
        img += int(strength)
    elif kind == "dark":
        img -= int(strength)
    elif kind == "gradient":
        ramp = _np.linspace(-strength, strength, w, dtype=_np.float32)
        img += ramp[None, :, None].astype(_np.int16)
    elif kind == "shadow":
        # A band across the lower-left: the robot half in shade, half in sun,
        # which is the normal state of an outdoor course with trees on it.
        m = _np.zeros((h, w), _np.float32)
        ys, xs = _np.mgrid[0:h, 0:w]
        m[(ys > h * 0.45) & (xs < w * 0.60 + (ys - h * 0.45) * 0.8)] = 1.0
        img -= (m[..., None] * strength * 1.4).astype(_np.int16)
    else:
        raise ValueError(f"unknown perturbation {kind!r}")
    return _np.clip(img, 0, 255).astype(_np.uint8)


def polys_to_mask(label_path, w, h, cls=LANE_CLASS):
    """Rasterise YOLO-seg polygons of one class into a boolean mask."""
    import cv2
    m = np.zeros((h, w), np.uint8)
    if not os.path.exists(label_path):
        return m.astype(bool)
    for line in open(label_path):
        parts = line.split()
        if len(parts) < 7 or int(float(parts[0])) != cls:
            continue
        xy = np.array(parts[1:], np.float64).reshape(-1, 2)
        xy[:, 0] *= w
        xy[:, 1] *= h
        cv2.fillPoly(m, [xy.astype(np.int32)], 1)
    return m.astype(bool)


def model_lane_mask(res, w, h, names, cls_name="lane"):
    import cv2
    m = np.zeros((h, w), np.uint8)
    if res.masks is None or res.boxes is None or not len(res.boxes):
        return m.astype(bool)
    ids = res.boxes.cls.cpu().numpy().astype(int)
    data = res.masks.data.cpu().numpy()
    for i, cid in enumerate(ids):
        if names.get(int(cid), str(cid)) != cls_name or i >= len(data):
            continue
        m |= (cv2.resize(data[i], (w, h), interpolation=cv2.INTER_NEAREST) > 0.5)
    return m.astype(bool)


def selftest():
    print("=" * 72)
    print("eval_recall self-test -- scoring maths only, no model needed")
    print("=" * 72)
    fails, run = [], 0

    def ck(name, got, want, tol=1e-9):
        nonlocal run
        run += 1
        ok = (abs(got - want) <= tol) if want == want else (got != got)
        print("  %-46s got %-10s want %-10s %s"
              % (name, round(got, 4) if got == got else "nan",
                 round(want, 4) if want == want else "nan",
                 "ok" if ok else "FAIL"))
        if not ok:
            fails.append(name)

    t = np.zeros((10, 10), bool); t[2:6, 2:6] = True        # 16 true px
    p = np.zeros((10, 10), bool); p[4:8, 2:6] = True        # 16 pred, 8 overlap
    tp, fp, fn = confusion(p, t)
    ck("TP", tp, 8); ck("FP", fp, 8); ck("FN", fn, 8)
    pr, rc, iou = prf(tp, fp, fn)
    ck("precision", pr, 0.5); ck("recall", rc, 0.5); ck("IoU", iou, 8 / 24)

    ck("perfect recall", prf(*confusion(t, t))[1], 1.0)
    ck("empty prediction -> recall 0", prf(*confusion(np.zeros_like(t), t))[1], 0.0)
    ck("no ground truth -> recall nan",
       prf(*confusion(p, np.zeros_like(t)))[1], float("nan"))
    ck("no prediction and no truth -> precision nan",
       prf(*confusion(np.zeros_like(t), np.zeros_like(t)))[0], float("nan"))

    # range binning: a pixel must land in exactly one bin, and the bin totals
    # must sum to the overall total -- otherwise the table lies.
    x = np.tile(np.linspace(0.0, 5.0, 10), (10, 1))
    valid = np.ones_like(x, bool)
    edges = [0, 1, 2, 3, 4, 6]
    bins = range_bins(x, valid, edges)
    stacked = np.stack(bins)
    ck("every valid pixel in exactly one bin",
       float(np.abs(stacked.sum(0).astype(int) - 1).max()), 0.0)
    tot, per = accumulate(p, t, bins)
    ck("bin TP sums to overall TP", float(sum(c[0] for c in per)), float(tot[0]))
    ck("bin FN sums to overall FN", float(sum(c[2] for c in per)), float(tot[2]))

    EXPECTED = 23
    import numpy as _np
    base = _np.full((60, 80, 3), 100, _np.uint8)
    ck("perturb none is a no-op", float(_np.abs(
        perturb_image(base, "none").astype(int) - base.astype(int)).max()), 0.0)
    ck("bright raises the mean", float(perturb_image(base, "bright", 40).mean()), 140.0)
    ck("dark lowers the mean", float(perturb_image(base, "dark", 40).mean()), 60.0)
    ck("values stay in range after a big shift",
       float(perturb_image(base, "bright", 400).max()), 255.0)
    ck("dark clips at zero, not below",
       float(perturb_image(base, "dark", 400).min()), 0.0)
    g = perturb_image(base, "gradient", 40)
    ck("gradient is darker on the left than the right",
       bool(g[:, :5].mean() < g[:, -5:].mean()), True)
    ck("gradient leaves the overall mean about the same",
       round(float(g.mean()) - 100.0), 0)
    sh = perturb_image(base, "shadow", 50)
    ck("shadow darkens part of the frame", bool(sh.min() < 100), True)
    ck("shadow leaves part of the frame alone", bool(sh.max() == 100), True)
    ck("shadow does not touch the top rows",
       float(sh[:10].mean()), 100.0)
    ck("perturbation does not change the shape",
       sh.shape == base.shape, True)

    print("\n%d checks run (expected at least %d)" % (run, EXPECTED))
    if run < EXPECTED:
        fails.append("only %d checks ran" % run)
    print()
    if fails:
        print("FAILURES:")
        for f in fails:
            print("  -", f)
        return 1
    print("ALL CHECKS PASSED")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--data", help="dataset dir from autolabel.py")
    ap.add_argument("--split", default="val")
    ap.add_argument("--weights", default=os.path.expanduser("~/esda_models/lane_seg.pt"))
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="0")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--detector", default="yolo", choices=["yolo", "classic"],
                    help="'classic' scores lane_classic.py -- a threshold, no "
                         "network -- through exactly this same scoring code, "
                         "so the comparison is like for like.")
    ap.add_argument("--perturb", default="none", choices=list(PERTURBS),
                    help="bend the LIGHTING before detection, leaving the "
                         "geometry (and so the labels) untouched. The sim "
                         "lights uniformly; a real course has shade on it.")
    ap.add_argument("--perturb-strength", type=int, default=50)
    ap.add_argument("--classic-max-range", type=float, default=6.0,
                    help="ROI range for the classic detector. The scoring "
                         "bins stop at 6 m, but the NODE cuts at "
                         "projection.max_range (4.0), so pass 4.0 to score "
                         "the configuration that would actually ship.")
    ap.add_argument("--method", default="tophat",
                    help="classic detector method: otsu|value|adaptive|tophat")
    ap.add_argument("--dedup", action="store_true",
                    help="score each DISTINCT frame once. A wedged robot "
                         "emits byte-identical frames -- 216 of job 109's 672 "
                         "were one frozen pose repeated -- and without this "
                         "they weight the score toward whatever that one view "
                         "happened to contain.")
    ap.add_argument("--edges", default="0,1,2,3,4,6",
                    help="range bin edges in metres")
    a = ap.parse_args()

    if a.selftest:
        raise SystemExit(selftest())
    if not a.data:
        raise SystemExit("--data is required (or use --selftest)")

    import synth_gen as sg
    # Only the YOLO path needs ultralytics. The classic detector must stay
    # runnable on a machine with nothing but opencv -- that is half its appeal.
    YOLO = None
    if a.detector == "yolo":
        from ultralytics import YOLO
    print(sg.camera_summary())
    if a.perturb != "none":
        print("lighting perturbed: %s (strength %d) -- geometry and labels "
              "unchanged" % (a.perturb, a.perturb_strength))

    # One ground-range map for the whole dataset: the geometry is fixed.
    X, _Y, valid = sg.pixel_to_ground()
    edges = [float(e) for e in a.edges.split(",")]
    bins = range_bins(X, valid, edges)

    imgs = sorted(glob.glob(os.path.join(a.data, "images", a.split, "*.jpg")))
    if a.limit:
        imgs = imgs[:a.limit]
    if not imgs:
        raise SystemExit("no images under %s/images/%s" % (a.data, a.split))
    n_all = len(imgs)
    if a.dedup:
        import hashlib
        seen, keep = set(), []
        for ip in imgs:
            d = hashlib.md5(open(ip, "rb").read()).hexdigest()
            if d not in seen:
                seen.add(d)
                keep.append(ip)
        imgs = keep
        print("dedup: %d distinct of %d frames (%d duplicates dropped)"
              % (len(imgs), n_all, n_all - len(imgs)))

    import cv2, time
    if a.detector == "classic":
        import lane_classic
        model = names = None
        print("detector: lane_classic method=%s  ROI to %.1f m "
              "(no network, no weights)" % (a.method, a.classic_max_range))
    else:
        model = YOLO(a.weights)
        names = model.names if isinstance(model.names, dict) else dict(enumerate(model.names))
        print("detector: YOLO %s" % a.weights)

    tot = (0, 0, 0)
    per = [(0, 0, 0)] * len(bins)
    frames_with_truth = 0
    frames_found = 0
    pred_px = []
    ms = []
    for k, ip in enumerate(imgs):
        img = cv2.imread(ip)
        if a.perturb != "none":
            img = perturb_image(img, a.perturb, a.perturb_strength)
        h, w = img.shape[:2]
        lp = os.path.join(a.data, "labels", a.split,
                          os.path.splitext(os.path.basename(ip))[0] + ".txt")
        truth = polys_to_mask(lp, w, h)
        t0 = time.monotonic()
        if a.detector == "classic":
            pred = lane_classic.lane_mask(img, a.method,
                                          max_range=a.classic_max_range)
            ms.append((time.monotonic() - t0) * 1e3)
        else:
            res = model.predict(img, conf=a.conf, imgsz=a.imgsz,
                                device=a.device, verbose=False)[0]
            ms.append((time.monotonic() - t0) * 1e3)
            pred = model_lane_mask(res, w, h, names)
        pred_px.append(int(pred.sum()))
        if truth.any():
            frames_with_truth += 1
            if (pred & truth).any():
                frames_found += 1
        t, p_ = accumulate(pred, truth, bins)
        tot = add(tot, t)
        per = [add(x, y) for x, y in zip(per, p_)]
        if (k + 1) % 50 == 0:
            print("  %d/%d" % (k + 1, len(imgs)))

    print("\n%d frames, %d with lane paint in them" % (len(imgs), frames_with_truth))
    print("inference %.1f ms median (%.1f-%.1f)"
          % (float(np.median(ms)), float(np.min(ms)), float(np.max(ms))))
    # ALIGNMENT-FREE. Every number below this needs the labels to sit on the
    # paint; this one does not. It separates "the model sees nothing" from
    # "the labels are in the wrong place", which is the first thing to know
    # when a recall number comes back low.
    fired = sum(1 for n in pred_px if n > 0)
    print("MODEL    predicted lane pixels in %d/%d frames (%.1f%%), "
          "median %d px when it fired"
          % (fired, len(imgs), 100.0 * fired / max(1, len(imgs)),
             int(np.median([n for n in pred_px if n > 0])) if fired else 0))
    print("         (this one needs no ground truth -- it is true even if the "
          "labels are misaligned)")

    pr, rc, iou = prf(*tot)
    print("\nOVERALL  precision %.3f   recall %.3f   IoU %.3f" % (pr, rc, iou))
    if frames_with_truth:
        print("FRAME    found some paint in %d/%d frames that had any (%.1f%%)"
              % (frames_found, frames_with_truth,
                 100.0 * frames_found / frames_with_truth))

    print("\nBY RANGE -- this is the one that matters; carrots land at 1.75-2.75 m")
    print("  %-12s %-10s %-10s %-10s %s" % ("range", "recall", "precision", "IoU", "truth px"))
    for (lo, hi), c in zip(zip(edges[:-1], edges[1:]), per):
        p_, r_, i_ = prf(*c)
        print("  %-12s %-10s %-10s %-10s %d"
              % ("%.0f-%.0f m" % (lo, hi),
                 "%.3f" % r_ if r_ == r_ else "n/a",
                 "%.3f" % p_ if p_ == p_ else "n/a",
                 "%.3f" % i_ if i_ == i_ else "n/a", c[0] + c[2]))


if __name__ == "__main__":
    main()
