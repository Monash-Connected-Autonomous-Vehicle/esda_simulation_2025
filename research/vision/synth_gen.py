#!/usr/bin/env python3
"""
ESDA synthetic scene generator  —  labelled lane/cone frames, no Gazebo needed.

WHY THIS EXISTS
    Task 2 of CLAUDE_TASKS.md ("label 200-400 frames in Roboflow") is hours of
    manual polygon work and was the thing blocking the whole pipeline. This
    skips it: we RENDER the camera view ourselves, so we already know which
    pixel is lane and which is cone. Labels are exact, free, and unlimited.

WHY IT MATCHES THE SIM
    Two things are taken from the actual simulation, not invented:
      1. the textures  -- models/orange_igvc/materials/textures/ground.png and
         Construction_Barrel_Diffuse.png are the real assets Gazebo renders, so
         asphalt speckle, lane-paint white and barrel orange are pixel-correct.
      2. the camera    -- the ground<->pixel homography below is the exact
         inverse of the IPM in lane_seg_node.py, built from the verified xacro
         chain (see vision_config.yaml projection block). Train and deploy
         therefore share one geometry.

THE POINT OF THE EXERCISE
    Sampled from the real textures:
        lane paint        BGR (242, 242, 242)
        barrel white band BGR (235, 224, 216)
    They are the same colour. That is exactly why the old white-threshold
    detector tagged barrels as lane and inflated the costmap into a false wall.
    No threshold can fix it. Every frame here deliberately contains that
    ambiguity, and the white bands are always labelled `cone`, never `lane`,
    so the model is forced to learn shape and context instead of colour.

USAGE
    python3 synth_gen.py --out ~/esda_datasets/lane_seg --n-train 1200 --n-val 200
    python3 synth_gen.py --out /tmp/ds --n-train 40 --n-val 10 --preview 6

OUTPUT (ready for train.py / ultralytics, no Roboflow account needed)
    <out>/images/{train,val}/*.jpg
    <out>/labels/{train,val}/*.txt      # YOLOv8-seg polygons, normalised
    <out>/data.yaml                     # absolute path, class order lane,cone,drivable
"""
import argparse
import os
import random

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Camera model -- READ FROM vision_config.yaml, not hard-coded.
#
# This module is the single source of truth for two different things: the
# synthetic TRAINING images, and the ground-truth labels autolabel.py
# computes for real renderer frames. If it disagrees with the camera the
# robot actually has, the model is trained for a camera that does not exist
# AND is then scored against labels for a camera that does not exist. Both
# wrong, in the same direction, so nothing looks broken.
#
# That is not hypothetical. These constants sat at the ORIGINAL rig --
# fx 528.43, height 0.4775, pitch 0 -- for a day after the camera moved to
# 90 deg, 1.1 m and 20 deg down. Three separate files have held this
# geometry (camera.xacro, vision_config.yaml, here) and keeping three in
# step by hand has failed every time it has been tried.
#
# So: one file owns it, and the other two derive. The literals below are
# only a fallback for running this module without the config present.
# ---------------------------------------------------------------------------
import yaml as _yaml

_FALLBACK = dict(fx=528.43, fy=528.43, cx=320.0, cy=240.0,
                 cam_height=0.4775, cam_x_offset=-0.165423, cam_pitch_deg=0.0)


def _load_projection(path=None):
    # ESDA_VISION_CONFIG picks another camera without editing this file, e.g.
    # camera_front.yaml / camera_pole_top.yaml for the two sim mounts.
    path = (path or os.environ.get("ESDA_VISION_CONFIG")
            or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "vision_config.yaml"))
    try:
        with open(path) as fh:
            p = dict(_yaml.safe_load(fh)["projection"])
    except Exception:
        return dict(_FALLBACK), False
    out = dict(_FALLBACK)
    out.update({k: p[k] for k in _FALLBACK if k in p})
    return out, True


_PROJ, _FROM_CONFIG = _load_projection()

FX = float(_PROJ["fx"])
FY = float(_PROJ["fy"])
CX = float(_PROJ["cx"])
CY = float(_PROJ["cy"])
CAM_HEIGHT = float(_PROJ["cam_height"])
CAM_X_OFFSET = float(_PROJ["cam_x_offset"])
# The mount's own down-pitch. Callers may still pass an extra pitch_deg, which
# ADDS to this, so existing call sites keep working and a caller that passes
# nothing now gets the real mount instead of silently assuming level.
CAM_PITCH_DEG = float(_PROJ["cam_pitch_deg"])

# cx, cy are the image centre, so the sensor size follows from them.
W, H_IMG = int(round(2.0 * CX)), int(round(2.0 * CY))
MAX_X = 30.0             # clamp: ignore ground further than this


def camera_summary():
    src = "vision_config.yaml" if _FROM_CONFIG else "FALLBACK LITERALS"
    import math
    hfov = 2.0 * math.degrees(math.atan(CX / FX))
    return ("camera from %s: %dx%d, fx=%.2f (hfov %.1f deg), height %.3f m, "
            "mount pitch %.1f deg down, x offset %.4f m"
            % (src, W, H_IMG, FX, hfov, CAM_HEIGHT, CAM_PITCH_DEG, CAM_X_OFFSET))


CLASS_NAMES = ["lane", "cone", "drivable"]
LANE, CONE, DRIVABLE = 0, 1, 2


# ---------------------------------------------------------------------------
# Geometry: pixel <-> ground. Exact inverse of lane_seg_node.py's _project().
# ---------------------------------------------------------------------------
def _eff_pitch(pitch_deg):
    """The camera's real down-tilt: its mount plus any extra the caller adds.

    Callers pass pitch_deg for a transient tilt (a bump, a future IMU
    reading). The mount angle itself comes from the config, so a caller that
    passes nothing gets the REAL camera instead of silently assuming level
    -- which is what made every label wrong after the mount changed.
    """
    return CAM_PITCH_DEG + float(pitch_deg or 0.0)


def pixel_to_ground(pitch_deg=0.0):
    """For every pixel, the ground point (X fwd, Y left) in base_link metres.

    Returns (X, Y, valid). Pixels above the horizon, or beyond MAX_X, are
    marked invalid. This mirrors _project() in lane_seg_node.py exactly:
    ray -> pitch rotation -> intersect the z = -cam_height ground plane.
    """
    us, vs = np.meshgrid(np.arange(W, dtype=np.float64),
                         np.arange(H_IMG, dtype=np.float64))
    dx = (us - CX) / FX
    dy = (vs - CY) / FY
    dz = np.ones_like(dx)

    p = np.deg2rad(_eff_pitch(pitch_deg))
    cp, sp = np.cos(p), np.sin(p)
    dy2 = cp * dy + sp * dz
    dz2 = -sp * dy + cp * dz

    with np.errstate(divide="ignore", invalid="ignore"):
        t = CAM_HEIGHT / dy2           # distance scale along the ray
    valid = np.isfinite(t) & (t > 0)

    t = np.where(valid, t, 0.0)
    X = np.where(valid, t * dz2 + CAM_X_OFFSET, 0.0)
    Y = np.where(valid, -t * dx, 0.0)
    valid &= (X > 0.2) & (X < MAX_X)
    return X, Y, valid


def ground_to_pixel(X, Y, pitch_deg=0.0):
    """Forward projection, used for placing barrels. Accepts scalars/arrays."""
    p = np.deg2rad(_eff_pitch(pitch_deg))
    cp, sp = np.cos(p), np.sin(p)
    A = np.asarray(X, dtype=np.float64) - CAM_X_OFFSET
    # solve the pitch equations for the ray's normalised y, then u,v
    s = (CAM_HEIGHT * cp - A * sp) / (A * cp + CAM_HEIGHT * sp)
    t = CAM_HEIGHT / (cp * s + sp)
    u = CX + FX * (-np.asarray(Y, dtype=np.float64) / t)
    v = CY + FY * s
    return u, v


def ground_to_pixel_elevated(X, Y, height, pitch_deg=0.0):
    """Project a point `height` metres ABOVE the ground point (X, Y).

    Used for a barrel's top rim. Works in camera-centred coordinates:
    forward = X - cam_x_offset, left = Y, down = cam_height - height.
    """
    p = np.deg2rad(_eff_pitch(pitch_deg))
    cp, sp = np.cos(p), np.sin(p)
    fwd = np.asarray(X, dtype=np.float64) - CAM_X_OFFSET
    left = np.asarray(Y, dtype=np.float64)
    down = CAM_HEIGHT - np.asarray(height, dtype=np.float64)
    # rotate the camera by pitch about its x (right) axis
    z_o = cp * fwd + sp * down
    y_o = -sp * fwd + cp * down
    x_o = -left
    z_o = np.maximum(z_o, 1e-6)
    return CX + FX * x_o / z_o, CY + FY * y_o / z_o


# ---------------------------------------------------------------------------
# Asphalt plate: the real ground texture with its painted lines inpainted out,
# so we keep genuine sim speckle but draw our own randomised lane geometry.
# ---------------------------------------------------------------------------
def build_asphalt_plate(ground_png, cache="/tmp/esda_asphalt.png"):
    if os.path.isfile(cache):
        return cv2.imread(cache)
    g = cv2.imread(ground_png, cv2.IMREAD_COLOR)
    if g is None:
        raise SystemExit(f"could not read ground texture: {ground_png}")
    hsv = cv2.cvtColor(g, cv2.COLOR_BGR2HSV)
    paint = ((hsv[..., 1] < 60) & (hsv[..., 2] > 165)).astype(np.uint8)
    paint = cv2.dilate(paint, np.ones((7, 7), np.uint8))
    plate = cv2.inpaint(g, paint, 5, cv2.INPAINT_TELEA)
    cv2.imwrite(cache, plate)
    return plate


def sample_barrel_palette(barrel_png):
    """Pull the real orange / white-band / black-base colours out of the atlas."""
    b = cv2.imread(barrel_png, cv2.IMREAD_COLOR)
    if b is None:
        raise SystemExit(f"could not read barrel texture: {barrel_png}")
    hsv = cv2.cvtColor(b, cv2.COLOR_BGR2HSV)
    Hc, S, V = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    orange = (Hc >= 5) & (Hc <= 22) & (S > 120) & (V > 120)
    white = (S < 40) & (V > 150)
    black = V < 70
    return {
        "orange": b[orange].mean(0) if orange.any() else np.array([3.6, 89.6, 213.9]),
        "white": b[white].mean(0) if white.any() else np.array([235.0, 224.0, 216.0]),
        "black": b[black].mean(0) if black.any() else np.array([15.6, 19.1, 20.2]),
    }


# ---------------------------------------------------------------------------
# Lane geometry. A cubic centreline gives arbitrary heading + curvature, so the
# dataset is not tied to the IGVC course layout.
# ---------------------------------------------------------------------------
def random_lane(rng):
    return {
        "a0": rng.uniform(-1.1, 1.1),      # lateral offset at the robot
        "a1": rng.uniform(-0.30, 0.30),    # heading
        "a2": rng.uniform(-0.055, 0.055),  # curvature
        "a3": rng.uniform(-0.004, 0.004),  # curvature change
        "width": rng.uniform(2.4, 4.4),    # corridor width, metres
        "paint": rng.uniform(0.08, 0.17),  # painted line width, metres
        "dashed_l": rng.random() < 0.22,
        "dashed_r": rng.random() < 0.22,
        "dash_period": rng.uniform(1.2, 3.0),
        "dash_duty": rng.uniform(0.45, 0.7),
        "single": rng.random() < 0.12,     # only one line visible
        "drop_left": rng.random() < 0.5,
    }


def centreline(lane, X):
    return lane["a0"] + lane["a1"] * X + lane["a2"] * X ** 2 + lane["a3"] * X ** 3


def centreline_slope(lane, X):
    return lane["a1"] + 2 * lane["a2"] * X + 3 * lane["a3"] * X ** 2


def lane_masks(lane, X, Y, valid):
    """Painted-line mask and the drivable corridor mask, in image space."""
    Yc = centreline(lane, X)
    slope = centreline_slope(lane, X)
    norm = np.sqrt(1.0 + slope ** 2)        # lateral -> true perpendicular distance

    half = lane["width"] / 2.0

    # Anti-aliasing. A 0.1 m line at 20 m projects to well under one pixel, so a
    # hard per-pixel test breaks it into a dotted trail -- which then renders as
    # visible white with no label (the model would learn "white is not lane").
    # Widen the metric half-width so every line stays at least ~1.6 px solid.
    A = np.maximum(X - CAM_X_OFFSET, 1e-3)
    hp = np.maximum(lane["paint"] / 2.0, 0.8 * A / FX)

    def one_side(sign, dashed):
        d = np.abs((Y - (Yc + sign * half))) / norm
        m = (d < hp) & valid
        if dashed:
            phase = np.mod(X, lane["dash_period"]) / lane["dash_period"]
            m &= phase < lane["dash_duty"]
        return m

    left = one_side(+1, lane["dashed_l"])
    right = one_side(-1, lane["dashed_r"])
    if lane["single"]:
        if lane["drop_left"]:
            left = np.zeros_like(left)
        else:
            right = np.zeros_like(right)

    paint = left | right
    corridor = (np.abs(Y - Yc) / norm < half) & valid & ~paint
    return paint, corridor


# ---------------------------------------------------------------------------
# Barrels, drawn as shaded billboards with the sim's own colours.
# ---------------------------------------------------------------------------
def draw_barrel(img, Xp, Yp, diam, height, pal, rng, pitch_deg):
    """Composite one barrel. Returns its silhouette mask (uint8 0/1) or None."""
    uc, v_base = ground_to_pixel(Xp, Yp, pitch_deg)
    _, v_top = ground_to_pixel_elevated(Xp, Yp, height, pitch_deg)
    A = Xp - CAM_X_OFFSET
    half_w_px = FX * (diam / 2.0) / A

    if not np.isfinite([uc, v_base, v_top, half_w_px]).all():
        return None
    h_px = v_base - v_top
    if h_px < 12 or half_w_px < 3:          # too far to be a useful label
        return None
    if uc + half_w_px < 0 or uc - half_w_px > W or v_base < 0 or v_top > H_IMG:
        return None

    bw = int(np.ceil(half_w_px * 2)) + 2
    bh = int(np.ceil(h_px)) + 2
    if bw < 3 or bh < 6 or bw > 4 * W or bh > 4 * H_IMG:
        return None

    # ---- build the barrel sprite in its own little canvas ----
    sprite = np.zeros((bh, bw, 3), np.float32)
    alpha = np.zeros((bh, bw), np.float32)

    yy = np.linspace(0.0, 1.0, bh).reshape(-1, 1)     # 0 top .. 1 bottom
    xx = np.linspace(-1.0, 1.0, bw).reshape(1, -1)    # -1 left .. +1 right

    # silhouette: a traffic drum is a near-cylinder, slightly narrower at the
    # top with a gentle convex belly. (Not an hourglass -- an earlier version
    # inverted this sign and produced flared top/bottom rims.)
    taper = rng.uniform(0.08, 0.16)
    belly = rng.uniform(0.0, 0.05)
    prof = (1.0 - taper * (1.0 - yy)) + belly * np.sin(np.pi * np.clip(yy, 0, 1))
    inside = np.abs(xx) <= prof
    alpha[inside] = 1.0

    # cylindrical shading: brightest a little off-centre, falls off at the rim
    lit = rng.uniform(-0.45, 0.45)
    shade = np.sqrt(np.clip(1.0 - (xx - lit) ** 2 / 1.6, 0.02, 1.0))
    shade = 0.55 + 0.55 * shade
    shade = shade * rng.uniform(0.85, 1.15)

    body = np.array(pal["orange"], np.float32).reshape(1, 1, 3)
    band = np.array(pal["white"], np.float32).reshape(1, 1, 3)
    base = np.array(pal["black"], np.float32).reshape(1, 1, 3)

    col = np.repeat(np.repeat(body, bh, 0), bw, 1)

    # white reflective bands -- the whole reason this dataset exists
    n_bands = rng.choice([1, 2, 2, 2, 3])
    bt = rng.uniform(0.055, 0.105)
    centres = {1: [0.5], 2: [0.33, 0.66], 3: [0.25, 0.5, 0.75]}[int(n_bands)]
    jitter = rng.uniform(-0.04, 0.04)
    for c in centres:
        m = (np.abs(yy - (c + jitter)) < bt / 2).repeat(bw, 1)
        col[m] = band[0, 0]

    # black rubber base ring at the foot
    foot = rng.uniform(0.10, 0.18)
    m = (yy > 1.0 - foot).repeat(bw, 1)
    col[m] = base[0, 0]

    sprite = col * shade[..., None]
    sprite = np.clip(sprite, 0, 255)

    # soften the silhouette edge a touch so it is not a cardboard cutout
    alpha = cv2.GaussianBlur(alpha, (0, 0), max(0.6, bw * 0.012))

    # ---- paste into the frame ----
    x0 = int(round(uc - half_w_px))
    y0 = int(round(v_top))
    X0, Y0 = max(0, x0), max(0, y0)
    X1, Y1 = min(W, x0 + bw), min(H_IMG, y0 + bh)
    if X1 <= X0 or Y1 <= Y0:
        return None
    sx0, sy0 = X0 - x0, Y0 - y0
    sx1, sy1 = sx0 + (X1 - X0), sy0 + (Y1 - Y0)

    a = alpha[sy0:sy1, sx0:sx1][..., None]
    roi = img[Y0:Y1, X0:X1].astype(np.float32)
    img[Y0:Y1, X0:X1] = (roi * (1 - a) + sprite[sy0:sy1, sx0:sx1] * a).astype(np.uint8)

    mask = np.zeros((H_IMG, W), np.uint8)
    mask[Y0:Y1, X0:X1] = (alpha[sy0:sy1, sx0:sx1] > 0.5).astype(np.uint8)
    return mask


def draw_shadow(img, Xp, Yp, diam, pitch_deg, strength):
    """Soft contact shadow. Deliberately NOT labelled as anything."""
    uc, vb = ground_to_pixel(Xp, Yp, pitch_deg)
    A = Xp - CAM_X_OFFSET
    rx = FX * (diam * 0.62) / A
    ry = max(2.0, rx * 0.32)
    if not np.isfinite([uc, vb, rx, ry]).all() or rx < 2:
        return
    layer = np.zeros((H_IMG, W), np.float32)
    cv2.ellipse(layer, (int(uc), int(vb)), (int(rx), int(ry)), 0, 0, 360, 1.0, -1)
    layer = cv2.GaussianBlur(layer, (0, 0), max(1.5, rx * 0.25))
    img[:] = (img.astype(np.float32) * (1 - strength * layer[..., None])).astype(np.uint8)


# ---------------------------------------------------------------------------
# Masks -> YOLOv8-seg polygon lines
# ---------------------------------------------------------------------------
def mask_to_polys(mask, cls, min_area=70, eps_frac=0.009):
    out = []
    cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        if cv2.contourArea(c) < min_area:
            continue
        eps = eps_frac * cv2.arcLength(c, True)
        ap = cv2.approxPolyDP(c, eps, True).reshape(-1, 2).astype(np.float64)
        if len(ap) < 3:
            continue
        ap[:, 0] = np.clip(ap[:, 0] / W, 0, 1)
        ap[:, 1] = np.clip(ap[:, 1] / H_IMG, 0, 1)
        out.append(f"{cls} " + " ".join(f"{p:.6f}" for p in ap.reshape(-1)))
    return out


# ---------------------------------------------------------------------------
def render_frame(rng, plate, pal, pix, want_drivable=True):
    # Wider than before. A ramp like the ones in the stock world is ~8.5 deg,
    # and the robot pitches both ways going on and off it.
    pitch = rng.uniform(-4.0, 10.0)
    X, Y, valid = pix[pitch_round(pitch)]

    # ---- ground from the real asphalt texture ----
    ppm = rng.uniform(26.0, 42.0)           # texture pixels per metre
    ox, oy = rng.uniform(0, 900), rng.uniform(0, 900)
    th = rng.uniform(-np.pi, np.pi)
    ct, st = np.cos(th), np.sin(th)
    tu = (X * ct - Y * st) * ppm + ox
    tv = (X * st + Y * ct) * ppm + oy
    ph, pw = plate.shape[:2]
    map_x = np.mod(tu, pw - 1).astype(np.float32)
    map_y = np.mod(tv, ph - 1).astype(np.float32)
    img = cv2.remap(plate, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)

    # Region above the horizon. In Gazebo this is sky plus whatever distant
    # scenery happens to be there; a flat grey band would be an unrealistically
    # clean cue, so give it a gradient, noise and sometimes a dark skyline.
    above = ~valid
    if above.any():
        g0, g1 = rng.uniform(110, 205), rng.uniform(110, 205)
        col = np.linspace(g0, g1, H_IMG).reshape(-1, 1)
        band = np.repeat(np.repeat(col[..., None], W, 1), 3, 2).astype(np.float32)
        band += np.random.randn(H_IMG, W, 3).astype(np.float32) * rng.uniform(1, 5)
        if rng.random() < 0.45:                     # distant treeline / barricades
            cut = int(CY * rng.uniform(0.45, 0.95))
            band[:cut] *= rng.uniform(0.45, 0.8)
        img[above] = np.clip(band, 0, 255).astype(np.uint8)[above]
    img = cv2.GaussianBlur(img, (0, 0), 0.6)

    # ---- lanes ----
    lane = random_lane(rng)
    paint, corridor = lane_masks(lane, X, Y, valid)
    paint_col = np.array([241.5, 242.1, 242.1], np.float32) * rng.uniform(0.82, 1.03)
    # Worn paint as a SMOOTH low-frequency field, not per-pixel salt. Per-pixel
    # noise shatters a line into sub-threshold fragments, which then get drawn
    # but dropped by the polygon area filter -> visible white with no label.
    lf = np.random.rand(12, 16).astype(np.float32)
    lf = cv2.resize(lf, (W, H_IMG), interpolation=cv2.INTER_CUBIC)
    wear = lf > rng.uniform(0.0, 0.42)
    pm = paint & wear
    img[pm] = np.clip(paint_col + np.random.randn(int(pm.sum()), 3) * 5, 0, 255).astype(np.uint8)

    # ---- barrels ----
    n = rng.choice([0, 1, 2, 2, 3, 3, 4, 5, 6])
    placements = []
    for _ in range(int(n)):
        Xp = rng.uniform(1.1, 13.0)
        if rng.random() < 0.55:
            # the hard case: a barrel sitting right next to a painted line
            side = rng.choice([-1, 1])
            Yp = centreline(lane, Xp) + side * lane["width"] / 2 + rng.uniform(-0.55, 0.55)
        else:
            Yp = rng.uniform(-5.5, 5.5)
        placements.append((Xp, Yp, rng.uniform(0.42, 0.62), rng.uniform(0.75, 1.05)))

    placements.sort(key=lambda p: -p[0])     # painter's algorithm: far first

    shadow_strength = rng.uniform(0.12, 0.42)
    for (Xp, Yp, d, hh) in placements:
        draw_shadow(img, Xp, Yp, d, pitch, shadow_strength)

    cone_masks, union = [], np.zeros((H_IMG, W), np.uint8)
    for (Xp, Yp, d, hh) in placements:
        m = draw_barrel(img, Xp, Yp, d, hh, pal, rng, pitch)
        if m is None:
            continue
        # a nearer barrel occludes whatever was already behind it
        for i, prev in enumerate(cone_masks):
            cone_masks[i] = prev & ~m.astype(bool)
        cone_masks.append(m.astype(bool))
        union |= m

    # lane pixels hidden behind a barrel are not visible, so not labelled
    paint_vis = pm & ~union.astype(bool)
    corridor_vis = corridor & ~union.astype(bool)

    # ---- camera roll (ramps, uneven ground) ----
    # Mostly small, occasionally steep, sometimes none at all.
    if rng.random() < 0.55:
        roll = rng.uniform(-14.0, 14.0) if rng.random() < 0.25 \
            else rng.uniform(-6.0, 6.0)
        stack = [paint_vis, corridor_vis] + cone_masks
        img, rolled = apply_roll(img, stack, roll)
        paint_vis, corridor_vis = rolled[0], rolled[1]
        cone_masks = rolled[2:]

    # ---- photometric domain randomisation ----
    img = photometric(img, rng)

    # ---- labels ----
    lines = []
    lines += mask_to_polys(paint_vis, LANE, min_area=12, eps_frac=0.006)
    for cm in cone_masks:
        if cm.sum() >= 40:
            lines += mask_to_polys(cm, CONE, min_area=40)
    if want_drivable:
        lines += mask_to_polys(corridor_vis, DRIVABLE, min_area=900, eps_frac=0.014)
    return img, lines


def apply_roll(img, masks, deg):
    """Rotate frame and masks about the principal point by `deg`.

    For a pinhole camera a roll about the optical axis is exactly an in-plane
    rotation of the image, so this is geometrically correct rather than an
    approximation -- and rotating the masks with the same matrix keeps the
    labels exact.
    """
    M = cv2.getRotationMatrix2D((CX, CY), deg, 1.0)
    out_img = cv2.warpAffine(img, M, (W, H_IMG), flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_REPLICATE)
    out_masks = [
        cv2.warpAffine(m.astype(np.uint8), M, (W, H_IMG),
                       flags=cv2.INTER_NEAREST,
                       borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
        for m in masks
    ]
    return out_img, out_masks


def photometric(img, rng):
    f = img.astype(np.float32)
    f *= rng.uniform(0.72, 1.28)                       # exposure
    f = (f - 128) * rng.uniform(0.85, 1.2) + 128       # contrast
    f += rng.uniform(-14, 14)                          # brightness
    if rng.random() < 0.5:                             # slight colour cast
        f *= np.array([rng.uniform(0.95, 1.05) for _ in range(3)], np.float32)
    f = np.clip(f, 0, 255)
    if rng.random() < 0.6:
        f = cv2.GaussianBlur(f, (0, 0), rng.uniform(0.4, 1.5))
    f += np.random.randn(*f.shape).astype(np.float32) * rng.uniform(1.0, 6.0)
    img = np.clip(f, 0, 255).astype(np.uint8)
    if rng.random() < 0.5:                             # jpeg-ish artefacts
        q = int(rng.uniform(45, 92))
        ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
        if ok:
            img = cv2.imdecode(enc, cv2.IMREAD_COLOR)
    return img


# cache the (X, Y, valid) fields per quantised pitch -- recomputing per frame is
# the slowest part of the generator by far
_PIX_CACHE = {}


def pitch_round(p):
    return round(float(p) * 2) / 2.0


def prime_pixel_cache():
    for p in np.arange(-4.0, 10.01, 0.5):
        _PIX_CACHE[pitch_round(p)] = pixel_to_ground(pitch_round(p))
    return _PIX_CACHE


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Synthesise labelled ESDA lane/cone frames")
    here = os.path.dirname(os.path.abspath(__file__))
    tex = os.path.join(here, "..", "esda_simulation_2025", "models", "orange_igvc",
                       "materials", "textures")
    ap.add_argument("--textures", default=tex, help="folder with ground.png + Construction_Barrel_Diffuse.png")
    ap.add_argument("--out", default="~/esda_datasets/lane_seg")
    ap.add_argument("--n-train", type=int, default=1200)
    ap.add_argument("--n-val", type=int, default=200)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--drivable", action="store_true",
                    help="also emit the drivable-corridor class. OFF by default: the\n"
                         "routing block in vision_config.yaml has drivable\n"
                         "publish_points:false, so it costs training time and buys\n"
                         "nothing. data.yaml still declares all 3 names, so the model\n"
                         "keeps a 3-class head and classes.mode:full still loads.")
    ap.add_argument("--preview", type=int, default=0,
                    help="also write N label-overlay images to <out>/preview/")
    args = ap.parse_args()

    out = os.path.abspath(os.path.expanduser(args.out))
    gpng = os.path.join(args.textures, "ground.png")
    bpng = os.path.join(args.textures, "Construction_Barrel_Diffuse.png")

    print("[gen] building asphalt plate (inpainting the painted lines out)...")
    plate = build_asphalt_plate(gpng)
    pal = sample_barrel_palette(bpng)
    print(f"[gen] sampled sim colours: orange={pal['orange'].round(1)} "
          f"band={pal['white'].round(1)} base={pal['black'].round(1)}")
    print("[gen] priming camera projection cache...")
    pix = prime_pixel_cache()

    rng = random.Random(args.seed)
    np.random.seed(args.seed)

    for split, n in (("train", args.n_train), ("val", args.n_val)):
        idir = os.path.join(out, "images", split)
        ldir = os.path.join(out, "labels", split)
        os.makedirs(idir, exist_ok=True)
        os.makedirs(ldir, exist_ok=True)
        counts = {0: 0, 1: 0, 2: 0}
        for i in range(n):
            img, lines = render_frame(rng, plate, pal, pix,
                                      want_drivable=args.drivable)
            for ln in lines:
                counts[int(ln.split()[0])] += 1
            stem = f"{split}_{i:06d}"
            cv2.imwrite(os.path.join(idir, stem + ".jpg"), img,
                        [cv2.IMWRITE_JPEG_QUALITY, 92])
            with open(os.path.join(ldir, stem + ".txt"), "w") as f:
                f.write("\n".join(lines) + ("\n" if lines else ""))
            if (i + 1) % 100 == 0:
                print(f"[gen] {split} {i+1}/{n}")
        print(f"[gen] {split}: {n} images  instances "
              f"lane={counts[0]} cone={counts[1]} drivable={counts[2]}")

    with open(os.path.join(out, "data.yaml"), "w") as f:
        f.write(
            "# Generated by synth_gen.py. Absolute path on purpose: ultralytics\n"
            "# resolves a relative/~ path against its own datasets dir, not cwd.\n"
            f"path: {out}\n"
            "train: images/train\n"
            "val: images/val\n\n"
            "names:\n  0: lane\n  1: cone\n  2: drivable\n"
        )
    print(f"[gen] wrote {out}/data.yaml")

    if args.preview:
        pdir = os.path.join(out, "preview")
        os.makedirs(pdir, exist_ok=True)
        vdir = os.path.join(out, "images", "val")
        names = sorted(os.listdir(vdir))[: args.preview]
        for nm in names:
            img = cv2.imread(os.path.join(vdir, nm))
            lab = os.path.join(out, "labels", "val", nm.rsplit(".", 1)[0] + ".txt")
            ov = img.copy()
            colour = {0: (0, 255, 0), 1: (0, 128, 255), 2: (255, 128, 0)}
            for ln in open(lab):
                p = ln.split()
                if len(p) < 7:
                    continue
                c = int(p[0])
                pts = np.array(p[1:], np.float64).reshape(-1, 2)
                pts[:, 0] *= W
                pts[:, 1] *= H_IMG
                cv2.fillPoly(ov, [pts.astype(np.int32)], colour[c])
            cv2.imwrite(os.path.join(pdir, nm), cv2.addWeighted(img, 0.55, ov, 0.45, 0))
        print(f"[gen] wrote {args.preview} label previews to {pdir}")


if __name__ == "__main__":
    main()
