#!/usr/bin/env python3
"""Side-by-side figure: camera view + bird's-eye, raw fragments vs linked lines.

The bird's-eye panel is the one that actually shows the association: each
reconstructed lane line gets its own colour, raw detections are grey dots, and
the bridged gaps are the stretches where a coloured curve runs with no grey
underneath it.
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lane_link as LL
import synth_gen as S
from test_lane_link import build_eval_frames, true_lines

BEV_W, BEV_H = 430, 460
X_LO, X_HI = 0.0, 9.0
Y_LO, Y_HI = -4.5, 4.5
COLS = [(60, 220, 60), (0, 190, 255), (255, 80, 220), (255, 230, 0)]


def to_bev(xy):
    x = np.asarray(xy, float).reshape(-1, 2)
    u = (( x[:, 1] - Y_LO) / (Y_HI - Y_LO) * BEV_W)
    v = (BEV_H - (x[:, 0] - X_LO) / (X_HI - X_LO) * BEV_H)
    return np.stack([u, v], 1)


def draw_bev(res, gp, lane):
    img = np.full((BEV_H, BEV_W, 3), 28, np.uint8)
    # range rings and the robot
    for r in (2, 4, 6, 8):
        p = to_bev([[r, Y_LO]])[0], to_bev([[r, Y_HI]])[0]
        cv2.line(img, tuple(p[0].astype(int)), tuple(p[1].astype(int)), (55, 55, 55), 1)
        cv2.putText(img, f"{r}m", (4, int(p[0][1]) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (110, 110, 110), 1)
    o = to_bev([[0, 0]])[0].astype(int)
    cv2.circle(img, tuple(o), 5, (200, 200, 200), -1)

    # ground truth, faint
    xs = np.arange(0.8, 8.5, 0.05)
    for ys in true_lines(lane, xs):
        p = to_bev(np.stack([xs, ys], 1)).astype(np.int32)
        cv2.polylines(img, [p], False, (80, 80, 80), 1, cv2.LINE_AA)

    # raw detections
    for q in to_bev(gp).astype(int):
        if 0 <= q[0] < BEV_W and 0 <= q[1] < BEV_H:
            cv2.circle(img, tuple(q), 1, (150, 150, 150), -1)

    # reconstructed lines
    for gid, poly in enumerate(res.polylines):
        if poly is None or len(poly) < 2:
            continue
        c = COLS[gid % len(COLS)]
        p = to_bev(poly).astype(np.int32)
        cv2.polylines(img, [p], False, c, 2, cv2.LINE_AA)
    # fragment centroids, coloured by the line they joined
    for fi, f in enumerate(res.fragments):
        gid = res.group_of_fragment.get(fi, 0)
        c = COLS[gid % len(COLS)]
        q = to_bev(f.centroid.reshape(1, 2))[0].astype(int)
        cv2.circle(img, tuple(q), 5, (20, 20, 20), -1)
        cv2.circle(img, tuple(q), 4, c, -1)

    cv2.putText(img, f"{len(res.fragments)} fragments -> {len(res.groups)} lane lines",
                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (235, 235, 235), 1, cv2.LINE_AA)
    cv2.putText(img, "grey = raw detections   colour = reconstructed",
                (8, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (150, 150, 150), 1, cv2.LINE_AA)
    return img


def main(weights, n=3):
    here = os.path.dirname(os.path.abspath(__file__))
    tex = os.environ.get("ESDA_TEXTURES", os.path.join(
        here, "..", "esda_simulation_2025", "models", "orange_igvc",
        "materials", "textures"))
    plate = S.build_asphalt_plate(os.path.join(tex, "ground.png"))
    pal = S.sample_barrel_palette(os.path.join(tex, "Construction_Barrel_Diffuse.png"))
    frames = build_eval_frames(14, plate, pal, seed=4242)
    from ultralytics import YOLO
    model = YOLO(weights)

    rows = []
    for (img, lane, Xf, Yf, valid) in frames:
        r = model.predict(img, imgsz=416, conf=0.35, device="cpu", verbose=False)[0]
        h, w = img.shape[:2]
        lane_px = np.zeros((h, w), bool)
        if r.masks is not None and r.boxes is not None:
            for m, c in zip(r.masks.data.cpu().numpy(),
                            r.boxes.cls.cpu().numpy().astype(int)):
                if c == 0:
                    lane_px |= cv2.resize(m, (w, h),
                                          interpolation=cv2.INTER_NEAREST) > 0.5
        sel = lane_px & valid
        if sel.sum() < 60:
            continue
        gp = np.stack([Xf[sel], Yf[sel]], 1)
        res = LL.link_lanes(gp)
        if res.bridged < 2:            # show frames where linking did work
            continue

        cam = img.copy()
        cam[sel] = (0.45 * cam[sel] + 0.55 * np.array([150, 150, 150])).astype(np.uint8)
        for gid, poly in enumerate(res.polylines):
            if poly is None:
                continue
            c = COLS[gid % len(COLS)]
            pts = []
            for (X, Y) in poly:
                A = X + 0.165423
                if A <= 0.05:
                    continue
                sray = 0.4775 / A
                u = 320.0 - 528.43 * Y / A
                v = 240.0 + 528.43 * sray
                if 0 <= u < w and 0 <= v < h:
                    pts.append([u, v])
            if len(pts) > 1:
                cv2.polylines(cam, [np.array(pts, np.int32)], False, c, 2, cv2.LINE_AA)
        cam = cv2.resize(cam, (int(BEV_H * 640 / 480), BEV_H))
        cv2.putText(cam, "camera: grey = raw lane mask, colour = linked line",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        rows.append(np.hstack([cam, draw_bev(res, gp, lane)]))
        if len(rows) >= n:
            break

    out = np.vstack(rows)
    bar = np.full((44, out.shape[1], 3), 18, np.uint8)
    cv2.putText(bar, "ESDA lane linking - dashed + barrel-occluded lanes rebuilt into continuous boundaries",
                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite('/mnt/user-data/outputs/esda_lane_linking.png', np.vstack([bar, out]))
    print("wrote /mnt/user-data/outputs/esda_lane_linking.png")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1
         else "/home/claude/esda/runs/lane_seg_cpu/weights/best.pt")
