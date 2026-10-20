#!/usr/bin/env python3
"""The decisive test: a white-striped barrel sitting ON a painted lane line.

This is the exact situation that broke the old white-threshold detector -- it
tagged the barrel's reflective bands as `lane`, which inflated the Nav2 costmap
into a false wall. We measure, on held-out frames built to be maximally
confusing, how much of each barrel's pixels the model assigns to `lane`.

PASS = barrels are called `cone`, and almost none of their area leaks into the
lane mask.
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import synth_gen as S


def build_hard_frames(n, plate, pal, seed=4242):
    """Frames where every barrel is deliberately placed on top of a lane line."""
    import random
    rng = random.Random(seed)
    np.random.seed(seed)
    pix = S.prime_pixel_cache()
    out = []
    for _ in range(n):
        pitch = rng.uniform(-1.0, 5.0)
        X, Y, valid = pix[S.pitch_round(pitch)]

        ppm = rng.uniform(26.0, 42.0)
        tu = X * ppm + rng.uniform(0, 900)
        tv = Y * ppm + rng.uniform(0, 900)
        ph, pw = plate.shape[:2]
        img = cv2.remap(plate, np.mod(tu, pw - 1).astype(np.float32),
                        np.mod(tv, ph - 1).astype(np.float32),
                        cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        img[~valid] = np.uint8(rng.uniform(120, 190))

        lane = S.random_lane(rng)
        lane["single"] = False
        paint, _ = S.lane_masks(lane, X, Y, valid)
        img[paint] = np.clip(np.array([241.5, 242.1, 242.1])
                             + np.random.randn(int(paint.sum()), 3) * 5, 0, 255).astype(np.uint8)

        # every barrel straddles a painted line
        truth = np.zeros((S.H_IMG, S.W), np.uint8)
        places = []
        for _ in range(rng.choice([2, 3, 4])):
            Xp = rng.uniform(1.3, 8.0)
            side = rng.choice([-1, 1])
            Yp = S.centreline(lane, Xp) + side * lane["width"] / 2 + rng.uniform(-0.12, 0.12)
            places.append((Xp, Yp, rng.uniform(0.44, 0.60), rng.uniform(0.78, 1.0)))
        places.sort(key=lambda p: -p[0])
        for (Xp, Yp, d, hh) in places:
            S.draw_shadow(img, Xp, Yp, d, pitch, rng.uniform(0.15, 0.35))
        for (Xp, Yp, d, hh) in places:
            m = S.draw_barrel(img, Xp, Yp, d, hh, pal, rng, pitch)
            if m is not None:
                truth |= m
        img = S.photometric(img, rng)
        out.append((img, truth.astype(bool), paint & ~truth.astype(bool)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--imgsz", type=int, default=416)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--save", default="/tmp/look/hard")
    a = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    tex = os.environ.get("ESDA_TEXTURES", os.path.join(
        here, "..", "esda_simulation_2025", "models", "orange_igvc",
        "materials", "textures"))
    plate = S.build_asphalt_plate(os.path.join(tex, "ground.png"))
    pal = S.sample_barrel_palette(os.path.join(tex, "Construction_Barrel_Diffuse.png"))
    frames = build_hard_frames(a.n, plate, pal)

    from ultralytics import YOLO
    model = YOLO(a.weights)

    tot_cone_px = tot_cone_as_lane = tot_cone_as_cone = 0
    tot_lane_px = tot_lane_found = 0
    os.makedirs(a.save, exist_ok=True)

    for i, (img, cone_t, lane_t) in enumerate(frames):
        r = model.predict(img, imgsz=a.imgsz, conf=a.conf, device="cpu", verbose=False)[0]
        h, w = img.shape[:2]
        pred = {0: np.zeros((h, w), bool), 1: np.zeros((h, w), bool)}
        if r.masks is not None and r.boxes is not None:
            md = r.masks.data.cpu().numpy()
            cid = r.boxes.cls.cpu().numpy().astype(int)
            for m, c in zip(md, cid):
                if c in pred:
                    pred[c] |= cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST) > 0.5

        tot_cone_px += int(cone_t.sum())
        tot_cone_as_lane += int((cone_t & pred[0]).sum())
        tot_cone_as_cone += int((cone_t & pred[1]).sum())
        tot_lane_px += int(lane_t.sum())
        tot_lane_found += int((lane_t & pred[0]).sum())

        if i < 6:
            ov = img.copy()
            ov[pred[0]] = (0, 255, 0)
            ov[pred[1]] = (0, 128, 255)
            cv2.imwrite(os.path.join(a.save, f"hard_{i}.jpg"),
                        cv2.addWeighted(img, 0.5, ov, 0.5, 0))

    leak = 100.0 * tot_cone_as_lane / max(1, tot_cone_px)
    hit = 100.0 * tot_cone_as_cone / max(1, tot_cone_px)
    rec = 100.0 * tot_lane_found / max(1, tot_lane_px)
    print(f"frames tested              : {a.n} (every barrel on a painted line)")
    print(f"barrel pixels              : {tot_cone_px}")
    print(f"  -> predicted CONE        : {hit:5.1f}%   (want high)")
    print(f"  -> predicted LANE (leak) : {leak:5.1f}%   (want ~0 -- this is the bug)")
    print(f"lane-paint pixels recalled : {rec:5.1f}%")
    print()
    print("VERDICT:", "PASS - cones are not being painted as lane"
          if leak < 2.0 and hit > 60.0 else "NEEDS WORK")


if __name__ == "__main__":
    main()
