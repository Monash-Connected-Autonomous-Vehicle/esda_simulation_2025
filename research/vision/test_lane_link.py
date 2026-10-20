#!/usr/bin/env python3
"""Verification for lane_link.py.

Part A — pure geometry, no model. Deterministic cases with a known answer.
Part B — end to end: trained model -> lane mask -> ground -> linker, on frames
         with dashed markings and barrels standing on the lines. Ground truth
         comes free because we generated the scene, so we know the exact
         equation of each true lane line.

Two different failure modes are measured, and they pull in opposite
directions:
    UNDER-LINKING  dashes of one line left in separate groups  -> recall
    OVER-LINKING   left and right boundaries merged into one   -> precision
Over-linking is the dangerous one: it invents a wall across the corridor.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lane_link as LL
import synth_gen as S


# ---------------------------------------------------------------- Part A ----
def dashed_line(x0, x1, y_of_x, period, duty, step=0.04, jitter=0.0, rng=None):
    xs = np.arange(x0, x1, step)
    keep = (np.mod(xs, period) / period) < duty
    xs = xs[keep]
    ys = np.array([y_of_x(x) for x in xs])
    if jitter and rng is not None:
        ys = ys + rng.normal(0, jitter, len(ys))
    return np.stack([xs, ys], 1)


def part_a():
    print("=" * 66)
    print("PART A - geometry only (no model)")
    print("=" * 66)
    rng = np.random.default_rng(0)
    ok = True

    # A1: one dashed straight line must come back as ONE group
    p = dashed_line(1.0, 8.0, lambda x: 0.8, 1.6, 0.5, jitter=0.01, rng=rng)
    r = LL.link_lanes(p)
    n = len(r.groups)
    print(f"A1 single dashed line      : {len(r.fragments)} fragments -> {n} group(s)"
          f"   {'PASS' if n == 1 else 'FAIL'}")
    ok &= (n == 1)

    # A2: TWO parallel dashed lines 3 m apart must stay SEPARATE
    pl = dashed_line(1.0, 8.0, lambda x: +1.5, 1.6, 0.5, jitter=0.01, rng=rng)
    pr = dashed_line(1.0, 8.0, lambda x: -1.5, 1.6, 0.5, jitter=0.01, rng=rng)
    r = LL.link_lanes(np.concatenate([pl, pr]))
    n = len(r.groups)
    print(f"A2 two parallel lines 3m   : {len(r.fragments)} fragments -> {n} group(s)"
          f"   {'PASS' if n == 2 else 'FAIL'}")
    ok &= (n == 2)

    # A3: a curving dashed line must still be one group (local headings)
    p = dashed_line(1.0, 8.0, lambda x: 0.045 * x * x - 0.25 * x, 1.5, 0.5,
                    jitter=0.01, rng=rng)
    r = LL.link_lanes(p)
    n = len(r.groups)
    print(f"A3 curved dashed line      : {len(r.fragments)} fragments -> {n} group(s)"
          f"   {'PASS' if n == 1 else 'FAIL'}")
    ok &= (n == 1)

    # A4: a solid line with a 0.9 m hole punched in it (a barrel) -> one group,
    #     and the fitted polyline must actually span the hole
    p = dashed_line(1.0, 8.0, lambda x: -0.4, 100.0, 1.0, jitter=0.01, rng=rng)
    p = p[(p[:, 0] < 3.6) | (p[:, 0] > 4.5)]
    r = LL.link_lanes(p)
    n = len(r.groups)
    filled = 0
    if r.polylines and r.polylines[0] is not None:
        pl_ = r.polylines[0]
        filled = int(((pl_[:, 0] > 3.7) & (pl_[:, 0] < 4.4)).sum())
    print(f"A4 occluded solid line     : {len(r.fragments)} fragments -> {n} group(s), "
          f"{filled} pts inside the hole   {'PASS' if n == 1 and filled > 5 else 'FAIL'}")
    ok &= (n == 1 and filled > 5)

    # A5: two COLLINEAR lines far apart (8 m) must NOT be joined
    pa = dashed_line(1.0, 2.5, lambda x: 0.0, 100.0, 1.0)
    pb = dashed_line(8.0, 8.9, lambda x: 0.0, 100.0, 1.0)
    r = LL.link_lanes(np.concatenate([pa, pb]))
    n = len(r.groups)
    print(f"A5 collinear, 5.5m gap     : {len(r.fragments)} fragments -> {n} group(s)"
          f"   {'PASS' if n == 2 else 'FAIL'}  (max_gap={LL.DEFAULTS['max_gap']})")
    ok &= (n == 2)

    # A6: a dash gap is nearly collinear, so the bow guard must NOT fire.
    #     Two 1 m fragments with a 1.4 m gap, dead straight.
    pa = dashed_line(1.0, 2.0, lambda x: 0.0, 100.0, 1.0)
    pb = dashed_line(3.4, 4.4, lambda x: 0.0, 100.0, 1.0)
    r = LL.link_lanes(np.concatenate([pa, pb]))
    spans = (len(r.groups) == 1 and r.rejected_bridges == 0
             and r.polylines[0] is not None
             and ((r.polylines[0][:, 0] > 2.2) & (r.polylines[0][:, 0] < 3.2)).sum() > 5)
    print(f"A6 dash gap, straight      : {len(r.groups)} group(s), "
          f"{r.rejected_bridges} bridge(s) rejected   {'PASS' if spans else 'FAIL'}")
    ok &= spans

    # A7: the guard itself, on a curve of known shape -- not via link_lanes,
    #     which may simply refuse to merge and would let this pass without
    #     ever exercising the thing under test.
    #
    #     Points only at the two ends, a quadratic through them, and a gap in
    #     the middle. A flat fit bows by nothing and must be kept; a strongly
    #     curved one bows well past 0.25 m and must be dropped.
    import numpy as _np
    def bow_of(curvature):
        xs = _np.concatenate([_np.linspace(0, 1.2, 25), _np.linspace(3.0, 4.2, 25)])
        ys = curvature * (xs - 2.1) ** 2
        pts = _np.stack([xs, ys], 1)
        cur = LL.Curve(pts, 2)
        s_, _ = cur.project(pts)
        lo, hi, step = float(s_.min()), float(s_.max()), 0.025
        poly = cur.sample(lo, hi, step)
        ss = _np.arange(lo, hi + step, step)
        bows = LL.bridge_bows(cur, poly, ss, s_, 0.2)
        drop, nrej = LL.bridge_drop_mask(cur, poly, ss, s_, 0.2, LL.DEFAULTS["max_bridge_bow"])
        return (bows[0][2] if bows else 0.0), nrej, int(drop.sum())

    flat_bow, flat_rej, _ = bow_of(0.0)
    bent_bow, bent_rej, bent_dropped = bow_of(0.5)
    a7 = (flat_rej == 0 and bent_rej == 1 and bent_dropped > 0
          and bent_bow > LL.DEFAULTS["max_bridge_bow"] > flat_bow)
    print(f"A7 bow guard               : straight bridge bows {flat_bow:.3f} m "
          f"(kept), curved bows {bent_bow:.3f} m ({bent_dropped} pts dropped)"
          f"   {'PASS' if a7 else 'FAIL'}  (max_bridge_bow={LL.DEFAULTS['max_bridge_bow']})")
    ok &= a7

    return ok


# ---------------------------------------------------------------- Part B ----
def build_eval_frames(n, plate, pal, seed=991):
    import random
    rng = random.Random(seed)
    np.random.seed(seed)
    pix = S.prime_pixel_cache()
    frames = []
    for _ in range(n):
        pitch = 0.0                      # isolate the LINKER from pitch error
        X, Y, valid = pix[S.pitch_round(pitch)]
        ppm = rng.uniform(26.0, 42.0)
        ph, pw = plate.shape[:2]
        img = S.cv2.remap(plate,
                          np.mod(X * ppm + rng.uniform(0, 900), pw - 1).astype(np.float32),
                          np.mod(Y * ppm + rng.uniform(0, 900), ph - 1).astype(np.float32),
                          S.cv2.INTER_LINEAR, borderMode=S.cv2.BORDER_WRAP)
        img[~valid] = np.uint8(rng.uniform(120, 190))

        lane = S.random_lane(rng)
        lane["single"] = False
        lane["dashed_l"] = True          # force the hard case
        lane["dashed_r"] = rng.random() < 0.5
        lane["dash_period"] = rng.uniform(1.2, 2.6)
        lane["dash_duty"] = rng.uniform(0.45, 0.65)
        lane["width"] = rng.uniform(2.6, 4.2)

        paint, _ = S.lane_masks(lane, X, Y, valid)
        img[paint] = np.clip(np.array([241.5, 242.1, 242.1])
                             + np.random.randn(int(paint.sum()), 3) * 5,
                             0, 255).astype(np.uint8)

        places = []
        for _ in range(rng.choice([1, 2, 3])):
            Xp = rng.uniform(1.5, 7.0)
            side = rng.choice([-1, 1])
            Yp = S.centreline(lane, Xp) + side * lane["width"] / 2 + rng.uniform(-0.1, 0.1)
            places.append((Xp, Yp, rng.uniform(0.44, 0.58), rng.uniform(0.78, 1.0)))
        places.sort(key=lambda p: -p[0])
        for (Xp, Yp, d, hh) in places:
            S.draw_shadow(img, Xp, Yp, d, pitch, rng.uniform(0.15, 0.35))
        for (Xp, Yp, d, hh) in places:
            S.draw_barrel(img, Xp, Yp, d, hh, pal, rng, pitch)
        img = S.photometric(img, rng)
        frames.append((img, lane, X, Y, valid))
    return frames


def true_lines(lane, xs):
    yc = S.centreline(lane, xs)
    half = lane["width"] / 2.0
    return yc + half, yc - half          # left, right


def part_b(weights, n=25):
    print()
    print("=" * 66)
    print("PART B - trained model -> ground -> linker")
    print("=" * 66)
    here = os.path.dirname(os.path.abspath(__file__))
    tex = os.environ.get("ESDA_TEXTURES", os.path.join(
        here, "..", "esda_simulation_2025", "models", "orange_igvc",
        "materials", "textures"))
    plate = S.build_asphalt_plate(os.path.join(tex, "ground.png"))
    pal = S.sample_barrel_palette(os.path.join(tex, "Construction_Barrel_Diffuse.png"))
    frames = build_eval_frames(n, plate, pal)

    from ultralytics import YOLO
    model = YOLO(weights)

    TP = FP = FN = 0
    cov_raw = cov_lnk = cov_tot = 0
    span_raw, span_lnk, span_tot = [0], [0], [0]
    groups_per_frame = []
    bridged = 0

    for (img, lane, Xf, Yf, valid) in frames:
        r = model.predict(img, imgsz=416, conf=0.35, device="cpu", verbose=False)[0]
        h, w = img.shape[:2]
        lane_px = np.zeros((h, w), bool)
        if r.masks is not None and r.boxes is not None:
            md = r.masks.data.cpu().numpy()
            cid = r.boxes.cls.cpu().numpy().astype(int)
            for m, c in zip(md, cid):
                if c == 0:
                    lane_px |= S.cv2.resize(m, (w, h),
                                            interpolation=S.cv2.INTER_NEAREST) > 0.5
        sel = lane_px & valid
        if sel.sum() < 30:
            continue
        gp = np.stack([Xf[sel], Yf[sel]], 1)

        res = LL.link_lanes(gp)
        if not res.fragments:
            continue
        groups_per_frame.append(len(res.groups))
        bridged += res.bridged

        # label each fragment by the true line it lies on
        labels = []
        for f in res.fragments:
            xs = f.pts[:, 0]
            yl, yr = true_lines(lane, xs)
            dl = np.abs(f.pts[:, 1] - yl).mean()
            dr = np.abs(f.pts[:, 1] - yr).mean()
            labels.append(0 if dl < dr else 1 if min(dl, dr) < 0.5 else -1)
            if min(dl, dr) >= 0.5:
                labels[-1] = -1

        idx = [i for i, l in enumerate(labels) if l >= 0]
        for ai in range(len(idx)):
            for bi in range(ai + 1, len(idx)):
                i, j = idx[ai], idx[bi]
                same_true = labels[i] == labels[j]
                same_pred = res.group_of_fragment[i] == res.group_of_fragment[j]
                if same_true and same_pred:
                    TP += 1
                elif (not same_true) and same_pred:
                    FP += 1
                elif same_true and not same_pred:
                    FN += 1

        # Coverage of the true lines, raw points vs linked polylines.
        # Reported two ways, because they answer different questions:
        #   FULL   over the whole 1-8 m window. Bounded by how far the MODEL
        #          can see, so it is not purely a measure of the linker.
        #   SPAN   restricted to the along-track range the model actually
        #          detected. This isolates the thing linking is for: are the
        #          holes BETWEEN detections closed?
        xs = np.arange(1.0, 8.0, 0.05)
        yl, yr = true_lines(lane, xs)
        for ys in (yl, yr):
            samp = np.stack([xs, ys], 1)
            samp = samp[np.abs(samp[:, 1]) < 5.0]
            if len(samp) == 0 or len(gp) == 0:
                continue
            d_raw = np.linalg.norm(samp[:, None, :] - gp[None, :, :], axis=2).min(1)
            d_lnk = (np.linalg.norm(samp[:, None, :] - res.dense[None, :, :], axis=2).min(1)
                     if len(res.dense) else np.full(len(samp), 9e9))
            cov_tot += len(samp)
            cov_raw += int((d_raw < 0.25).sum())
            cov_lnk += int((d_lnk < 0.25).sum())

            near = d_raw < 0.5                       # where this line was seen
            if near.sum() >= 2:
                i0, i1 = np.where(near)[0][[0, -1]]
                sl = slice(i0, i1 + 1)
                span_tot[0] += (i1 - i0 + 1)
                span_raw[0] += int((d_raw[sl] < 0.25).sum())
                span_lnk[0] += int((d_lnk[sl] < 0.25).sum())

    prec = TP / max(1, TP + FP)
    rec = TP / max(1, TP + FN)
    print(f"frames evaluated            : {len(groups_per_frame)}")
    print(f"mean lane groups per frame  : {np.mean(groups_per_frame):.2f}  (2 is ideal)")
    print(f"fragment joins made         : {bridged}")
    print()
    print(f"pair precision (no bad merge): {prec:6.3f}   FP={FP}  <- left+right merged")
    print(f"pair recall    (dashes joined): {rec:6.3f}   FN={FN}  <- same line split")
    print()
    print(f"[FULL 1-8m]  covered RAW {100.0*cov_raw/max(1,cov_tot):5.1f}%"
          f"   LINKED {100.0*cov_lnk/max(1,cov_tot):5.1f}%"
          f"   ({100.0*(cov_lnk-cov_raw)/max(1,cov_tot):+.1f} pts)")
    sr = 100.0 * span_raw[0] / max(1, span_tot[0])
    sl_ = 100.0 * span_lnk[0] / max(1, span_tot[0])
    gain = sl_ - sr
    print(f"[DETECTED span] covered RAW {sr:5.1f}%   LINKED {sl_:5.1f}%"
          f"   ({gain:+.1f} pts)   <- what linking is for")
    print()
    print("VERDICT:", "PASS" if (prec > 0.95 and rec > 0.80 and gain > 5) else "NEEDS WORK")
    return prec, rec, gain


if __name__ == "__main__":
    a = part_a()
    w = sys.argv[1] if len(sys.argv) > 1 else \
        "/home/claude/esda/runs/lane_seg_cpu/weights/best.pt"
    part_b(w, n=int(sys.argv[2]) if len(sys.argv) > 2 else 25)
    print()
    print("Part A:", "PASS" if a else "FAIL")
