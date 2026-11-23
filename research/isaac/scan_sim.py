#!/usr/bin/env python3
"""Synthesise /scan_fused analytically: the numpy reference implementation.

WHY ANALYTIC RATHER THAN A REAL RAYCAST
    Lane markings are *paint*.  They have no geometry, no height and no
    collider, so there is nothing in the Isaac scene for a RayCaster to hit --
    yet they are more than half of what the robot actually navigates by, since
    nav2_params.yaml points every costmap and AMCL at /scan_fused rather than
    /scan.  Whatever produces lane returns has to be analytic regardless.

    Once the lane half is analytic, making the obstacle half analytic too buys
    one code path instead of two, no scene dependency at all, and a module that
    tools/validate_isaac_obs.py can check against a brute-force oracle with no
    Isaac and no ROS installed.  The cost is that this must agree with Isaac's
    own MultiMeshRayCaster, which is a test, not an assumption.

THE ASYMMETRY THAT MATTERS
    Obstacle returns come from a 360 degree lidar reaching 100 m.  Lane returns
    come from a camera: horizontal FOV 1.089 rad and a depth image clipped at
    10 m (description/camera.xacro).  So in the real /scan_fused there is no
    paint behind the robot, and none more than 10 m ahead, however much paint is
    on the ground.

    Getting this wrong is not a rounding error -- a policy trained on paint it
    can see behind itself learns to reverse along a lane boundary it would be
    blind to on the real robot.

FRAME NOTE
    /scan is published in laser_frame and the lane points are projected through
    camera_link, which sit about 0.24 m apart in x (lidar.xacro has the pole at
    chassis 0.560407, camera.xacro the front mount at 0.8).  Both are collapsed
    to a single robot-origin frame here.  At a 0.05 m costmap resolution that
    offset is five cells; it is modelled as a configurable lever arm rather than
    silently ignored.

NOT IMPORTED BY THE ROS NODE
    The deployed node receives a real /scan_fused and never calls this.  It
    lives beside obs_core.py because both are pure numpy and the harness tests
    them together.
"""
import math

import numpy as np

from obs_core import (
    ANGLE_MAX,
    ANGLE_MIN,
    CAMERA_HALF_FOV,
    CAMERA_RANGE_MAX,
    CAMERA_RANGE_MIN,
    N_BEAMS_RAW,
    NOISE_STD,
    RANGE_MAX,
    RANGE_MIN,
    beam_angles,
)

EPS = 1e-12


class ScanCfg(object):
    """Sensor geometry. Defaults mirror description/lidar.xacro and camera.xacro."""

    def __init__(self, n_beams=N_BEAMS_RAW, angle_min=ANGLE_MIN, angle_max=ANGLE_MAX,
                 range_min=RANGE_MIN, range_max=RANGE_MAX,
                 lane_half_fov=CAMERA_HALF_FOV, lane_range_min=CAMERA_RANGE_MIN,
                 lane_range_max=CAMERA_RANGE_MAX, camera_offset_x=0.0):
        self.n_beams = int(n_beams)
        self.angle_min = float(angle_min)
        self.angle_max = float(angle_max)
        self.range_min = float(range_min)
        self.range_max = float(range_max)
        self.lane_half_fov = float(lane_half_fov)
        self.lane_range_min = float(lane_range_min)
        self.lane_range_max = float(lane_range_max)
        self.camera_offset_x = float(camera_offset_x)

    def angles(self):
        return np.linspace(self.angle_min, self.angle_max, self.n_beams, dtype=np.float64)


class ScanNoise(object):
    """Degradations that make a synthetic scan resemble a real /scan_fused.

    Not decoration.  The lidar half is nearly clean, but the lane half comes
    out of a learned detector and a fragment linker: research/vision measures
    0.921 recall on rejoining dashes of one line, and dropped fragments are the
    dominant failure.  A policy trained on perfect paint has never seen the
    input it will be given.

    range_std       lidar gaussian noise; 0.015 m in lidar.xacro
    dropout_p       per-beam loss (no return at all)
    lane_dropout_p  per-SEGMENT loss, applied before casting, so a whole stretch
                    of boundary disappears the way an unlinked fragment does --
                    which is different from, and worse than, losing scattered
                    beams
    spurious_p      per-beam false return at a uniformly random range
    range_bias_std  one constant offset per episode, for calibration error
    """

    def __init__(self, range_std=NOISE_STD, dropout_p=0.0, lane_dropout_p=0.0,
                 spurious_p=0.0, range_bias_std=0.0):
        self.range_std = float(range_std)
        self.dropout_p = float(dropout_p)
        self.lane_dropout_p = float(lane_dropout_p)
        self.spurious_p = float(spurious_p)
        self.range_bias_std = float(range_bias_std)


# ------------------------------------------------------------- primitives --
def ray_discs(origin, dirs, centres, radii, range_min, range_max):
    """Distance along each ray to the nearest disc. (B,) with inf for a miss.

    dirs must be unit length, which lets the quadratic drop its leading
    coefficient: |O + tD - C|^2 = R^2 becomes t^2 + 2bt + c = 0.
    """
    dirs = np.asarray(dirs, dtype=np.float64)
    centres = np.asarray(centres, dtype=np.float64).reshape(-1, 2)
    radii = np.asarray(radii, dtype=np.float64).reshape(-1)
    if centres.shape[0] == 0:
        return np.full(dirs.shape[0], np.inf)

    f = np.asarray(origin, dtype=np.float64)[None, :] - centres        # (K,2)
    b = dirs @ f.T                                                     # (B,K)
    c = (f * f).sum(axis=1) - radii * radii                            # (K,)
    disc = b * b - c[None, :]
    ok = disc >= 0.0
    sq = np.sqrt(np.where(ok, disc, 0.0))
    t_near = -b - sq
    t_far = -b + sq
    # Entering hit if it is in front of us, otherwise the exit point -- which
    # is the correct answer when the sensor sits inside a disc.
    t = np.where(t_near >= range_min, t_near, t_far)
    hit = ok & (t >= range_min) & (t <= range_max)
    return np.where(hit, t, np.inf).min(axis=1)


def ray_segments(origin, dirs, p0, p1, range_min, range_max):
    """Distance along each ray to the nearest segment. (B,) with inf for a miss.

    Solves O + t*D = P + u*E for t >= 0 and u in [0,1] by crossing with E and
    with D in turn, which avoids forming or inverting a 2x2 matrix per pair.
    """
    dirs = np.asarray(dirs, dtype=np.float64)
    p0 = np.asarray(p0, dtype=np.float64).reshape(-1, 2)
    p1 = np.asarray(p1, dtype=np.float64).reshape(-1, 2)
    if p0.shape[0] == 0:
        return np.full(dirs.shape[0], np.inf)

    e = p1 - p0                                                        # (S,2)
    g = p0 - np.asarray(origin, dtype=np.float64)[None, :]             # (S,2)
    denom = (dirs[:, None, 0] * e[None, :, 1]
             - dirs[:, None, 1] * e[None, :, 0])                       # (B,S)
    safe = np.where(np.abs(denom) < EPS, np.nan, denom)
    t = (g[None, :, 0] * e[None, :, 1] - g[None, :, 1] * e[None, :, 0]) / safe
    u = (g[None, :, 0] * dirs[:, None, 1] - g[None, :, 1] * dirs[:, None, 0]) / safe
    hit = (np.isfinite(t) & (t >= range_min) & (t <= range_max)
           & (u >= 0.0) & (u <= 1.0))
    return np.where(hit, t, np.inf).min(axis=1)


def cull_segments(origin, p0, p1, reach):
    """Keep only segments that could possibly be within `reach` of origin.

    A segment is kept when its midpoint is within reach plus half its own
    length, which is conservative -- it never drops a segment that could hit.
    Without it, a 570-sample centreline becomes ~1140 segments and the (898 x S)
    intermediates cost tens of megabytes for a single scan.
    """
    p0 = np.asarray(p0, dtype=np.float64).reshape(-1, 2)
    p1 = np.asarray(p1, dtype=np.float64).reshape(-1, 2)
    if p0.shape[0] == 0:
        return p0, p1, np.zeros(0, dtype=bool)
    mid = 0.5 * (p0 + p1)
    half = 0.5 * np.linalg.norm(p1 - p0, axis=1)
    d = np.linalg.norm(mid - np.asarray(origin, dtype=np.float64)[None, :], axis=1)
    keep = d <= (reach + half)
    return p0[keep], p1[keep], keep


# ---------------------------------------------------------------- helpers --
def polyline_segments(points, closed=True):
    """(N,2) polyline -> (p0, p1) segment endpoints."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if closed:
        return pts, np.roll(pts, -1, axis=0)
    return pts[:-1], pts[1:]


def boundaries_from_centreline(centreline, widths, closed=True):
    """Centreline plus per-sample width -> left and right boundary polylines.

    The normal is taken from the centred difference of neighbours, so a sample
    at a corner gets the average of the two incident directions rather than one
    of them.  Mirrors the convention in research/vision/make_worlds.offset_path.
    """
    pts = np.asarray(centreline, dtype=np.float64).reshape(-1, 2)
    w = np.asarray(widths, dtype=np.float64).reshape(-1)
    if pts.shape[0] != w.shape[0]:
        raise ValueError("centreline has %d points but %d widths"
                         % (pts.shape[0], w.shape[0]))
    if closed:
        nxt, prv = np.roll(pts, -1, axis=0), np.roll(pts, 1, axis=0)
    else:
        nxt = np.vstack([pts[1:], pts[-1:]])
        prv = np.vstack([pts[:1], pts[:-1]])
    tang = nxt - prv
    norm = np.linalg.norm(tang, axis=1, keepdims=True)
    tang = tang / np.where(norm < EPS, 1.0, norm)
    left_n = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    half = (0.5 * w)[:, None]
    return pts + left_n * half, pts - left_n * half


# ------------------------------------------------------------------- scan --
def synth_scan(pose, cfg=None, discs=None, disc_radii=None,
               lane_polylines=None, noise=None, rng=None):
    """A synthetic /scan_fused for one robot pose.

    pose           (x, y, yaw) in world metres
    discs          (K,2) obstacle centres; barrels and barricades
    disc_radii     (K,) radii
    lane_polylines list of (N,2) arrays -- the painted boundaries
    returns        (cfg.n_beams,) ranges, +inf where nothing was hit
    """
    cfg = cfg or ScanCfg()
    rng = rng or np.random.default_rng(0)
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])

    alpha = cfg.angles()
    world_ang = alpha + yaw
    dirs = np.stack([np.cos(world_ang), np.sin(world_ang)], axis=1)
    origin = np.array([x, y], dtype=np.float64)

    ranges = np.full(cfg.n_beams, np.inf, dtype=np.float64)

    if discs is not None and len(discs):
        ranges = np.minimum(ranges, ray_discs(origin, dirs, discs, disc_radii,
                                              cfg.range_min, cfg.range_max))

    if lane_polylines:
        cam_origin = origin + cfg.camera_offset_x * np.array([math.cos(yaw), math.sin(yaw)])
        p0s, p1s = [], []
        for poly in lane_polylines:
            a, b = polyline_segments(poly, closed=False)
            p0s.append(a)
            p1s.append(b)
        p0 = np.vstack(p0s) if p0s else np.zeros((0, 2))
        p1 = np.vstack(p1s) if p1s else np.zeros((0, 2))

        if noise is not None and noise.lane_dropout_p > 0.0 and p0.shape[0]:
            keep = rng.random(p0.shape[0]) >= noise.lane_dropout_p
            p0, p1 = p0[keep], p1[keep]

        p0, p1, _ = cull_segments(cam_origin, p0, p1, cfg.lane_range_max)
        if p0.shape[0]:
            lane = ray_segments(cam_origin, dirs, p0, p1,
                                cfg.lane_range_min, cfg.lane_range_max)
            # Paint exists in /scan_fused only inside the camera's cone.
            outside = np.abs(alpha) > cfg.lane_half_fov
            lane[outside] = np.inf
            ranges = np.minimum(ranges, lane)

    if noise is not None:
        ranges = apply_noise(ranges, cfg, noise, rng)
    return ranges


def apply_noise(ranges, cfg, noise, rng):
    """Range noise, per-beam dropout, spurious returns and a constant bias."""
    out = np.array(ranges, dtype=np.float64, copy=True)
    finite = np.isfinite(out)

    if noise.range_bias_std > 0.0:
        out[finite] += rng.normal(0.0, noise.range_bias_std)
    if noise.range_std > 0.0:
        out[finite] += rng.normal(0.0, noise.range_std, size=int(finite.sum()))
    if noise.dropout_p > 0.0:
        out[rng.random(out.shape[0]) < noise.dropout_p] = np.inf
    if noise.spurious_p > 0.0:
        hit = rng.random(out.shape[0]) < noise.spurious_p
        n = int(hit.sum())
        if n:
            out[hit] = rng.uniform(cfg.range_min, cfg.lane_range_max, size=n)

    out = np.where(np.isfinite(out), np.maximum(out, cfg.range_min), np.inf)
    return out


# --------------------------------------------------------------- oracles --
def brute_force_scan(pose, cfg=None, discs=None, disc_radii=None,
                     lane_polylines=None, step=0.01):
    """Deliberately stupid reference: march each ray and test as you go.

    This is the independent implementation tools/validate_isaac_obs.py compares
    the vectorised version against, in the manner of
    tools/validate_waypoint_logic.py -- which checks the fast clearance code
    against the original brute-force one so the question answered is "did the
    fast version change any decision", not merely "does it run".

    Accurate only to `step`, and far too slow for anything but a test.
    """
    cfg = cfg or ScanCfg()
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    alpha = cfg.angles()
    centres = (np.asarray(discs, dtype=np.float64).reshape(-1, 2)
               if discs is not None and len(discs) else np.zeros((0, 2)))
    radii = (np.asarray(disc_radii, dtype=np.float64).reshape(-1)
             if disc_radii is not None and len(centres) else np.zeros(0))

    segs = []
    for poly in (lane_polylines or []):
        a, b = polyline_segments(poly, closed=False)
        segs.append((a, b))

    out = np.full(cfg.n_beams, np.inf)
    for i, ang in enumerate(alpha):
        dx, dy = math.cos(ang + yaw), math.sin(ang + yaw)
        t = cfg.range_min
        while t <= cfg.range_max:
            px, py = x + t * dx, y + t * dy
            if len(centres):
                d = np.hypot(centres[:, 0] - px, centres[:, 1] - py)
                if np.any(d <= radii):
                    out[i] = t
                    break
            if segs and abs(ang) <= cfg.lane_half_fov and t <= cfg.lane_range_max:
                if any(_near_segment(px, py, a, b, step) for a, b in segs):
                    out[i] = t
                    break
            t += step
    return out


def _near_segment(px, py, p0, p1, tol):
    e = p1 - p0
    L2 = (e * e).sum(axis=1)
    L2 = np.where(L2 < EPS, 1.0, L2)
    u = np.clip(((px - p0[:, 0]) * e[:, 0] + (py - p0[:, 1]) * e[:, 1]) / L2, 0.0, 1.0)
    cx = p0[:, 0] + u * e[:, 0]
    cy = p0[:, 1] + u * e[:, 1]
    return bool(np.any(np.hypot(cx - px, cy - py) <= tol))
