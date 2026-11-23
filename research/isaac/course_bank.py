#!/usr/bin/env python3
"""Courses as fixed-shape arrays, ready to become per-environment tensors.

WHY A BANK OF ARRAYS
    Training needs thousands of robots on different courses at once. Spawning
    different course GEOMETRY per environment is expensive and awkward; but a
    scan-based policy never touches the course with anything except rays, and
    lane paint has no collider to touch in the first place. So the whole course
    reduces to numbers: a centreline, a corridor width, two boundary polylines
    and a list of obstacle discs.

    Every course is resampled to the same number of points and padded to the
    same obstacle count, so the bank is a rectangular array that uploads to the
    GPU once and is indexed per environment on reset. Swapping an environment
    to a different course is then an index change, not a scene edit.

WHERE THE COURSES COME FROM
    Generated ones reuse research/vision/make_worlds.py -- the same ten layouts
    (oval, wavy, figure8, slalom, spline, hairpin, crossroads, narrowing,
    zigzag, fork) that the vision work already drives, including its
    place_barrels, which deliberately puts barrels ON and BESIDE the painted
    lines rather than scattering them. Reusing it means the RL policy and the
    lane detector are trained against the same idea of what a course is.

    The real course comes from config/course_meta.yaml (centreline recovered
    from ground.png) and config/course_obstacles.yaml (colliders parsed from
    model.sdf).

FRAMES
    Everything is in the igvc.sdf world frame. make_worlds works in the model
    frame, so generated courses get the same (0, 16.3) offset the real course
    carries. One frame everywhere, so spawn poses and ground geometry do not
    need special cases.

NO ISAAC, NO TORCH
    Plain numpy, so tools/validate_isaac_obs.py can check it.
"""
import math
import os
import random

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
CONFIG = os.path.join(HERE, "config")

# igvc.sdf includes orange_igvc at (0, 16.3, 0).
MODEL_OFFSET = np.array([0.0, 16.3])

# Every course is resampled to this many centreline points. 256 over a ~135 m
# lap is about 0.5 m spacing -- finer than the 0.45 m robot radius, so the
# progress measure cannot skip past the robot.
N_SAMPLES = 256
# Obstacle slots per course. Padding is marked by a radius of zero, which
# raycasting treats as a miss, so no masking logic is needed downstream.
N_OBSTACLES = 64

DEFAULT_COURSES = ("oval", "wavy", "figure8", "slalom", "spline",
                   "hairpin", "crossroads", "narrowing", "zigzag", "fork")


# ------------------------------------------------------------------ helpers --
def resample_closed(points, n, closed=True):
    """Even arc-length resampling to exactly n points."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    ring = np.vstack([pts, pts[:1]]) if closed else pts
    seg = np.linalg.norm(np.diff(ring, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    if total <= 0:
        return np.repeat(pts[:1], n, axis=0), 0.0
    want = np.linspace(0.0, total, n, endpoint=closed is False) if not closed \
        else np.arange(n) * (total / n)
    out = np.stack([np.interp(want, s, ring[:, 0]),
                    np.interp(want, s, ring[:, 1])], axis=1)
    return out, total


def arclength(points, closed=True):
    """Cumulative distance along the polyline, and its total length."""
    pts = np.asarray(points, dtype=np.float64)
    ring = np.vstack([pts, pts[:1]]) if closed else pts
    seg = np.linalg.norm(np.diff(ring, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])[:len(pts)], float(seg.sum())


def boundaries(centre, widths, closed=True):
    """Left and right boundary polylines.

    Left means left OF THE DIRECTION OF TRAVEL, so on a counter-clockwise loop
    it is the inner one. tools/validate_isaac_obs.py pins that down, because
    swapping them mirrors every course the policy ever sees.
    """
    pts = np.asarray(centre, dtype=np.float64)
    w = np.asarray(widths, dtype=np.float64)
    if closed:
        nxt, prv = np.roll(pts, -1, axis=0), np.roll(pts, 1, axis=0)
    else:
        nxt = np.vstack([pts[1:], pts[-1:]])
        prv = np.vstack([pts[:1], pts[:-1]])
    tang = nxt - prv
    norm = np.linalg.norm(tang, axis=1, keepdims=True)
    tang = tang / np.where(norm < 1e-12, 1.0, norm)
    left_n = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    half = (0.5 * w)[:, None]
    return pts + left_n * half, pts - left_n * half


def heading_at(centre, idx, closed=True):
    """Tangent heading at a centreline index, for spawning the robot."""
    pts = np.asarray(centre, dtype=np.float64)
    j = (idx + 1) % len(pts) if closed else min(idx + 1, len(pts) - 1)
    d = pts[j] - pts[idx]
    return math.atan2(float(d[1]), float(d[0]))


# ----------------------------------------------------------------- courses --
def _make_worlds():
    """Import research/vision/make_worlds lazily; it pulls in cv2."""
    import sys
    path = os.path.join(REPO, "research", "vision")
    if path not in sys.path:
        sys.path.insert(0, path)
    import make_worlds
    return make_worlds


def generated_course(name, rng, n_samples=N_SAMPLES, n_obstacles=N_OBSTACLES):
    """One procedurally generated course, in world-frame metres."""
    mw = _make_worlds()
    if name not in mw.COURSES:
        raise KeyError("unknown course %r; have %s" % (name, sorted(mw.COURSES)))
    segments = mw.COURSES[name](rng)

    # Use the longest segment as the course. The branching layouts (fork,
    # crossroads) return several; the policy follows one corridor at a time and
    # the others become scenery the scan still sees.
    pts, closed, width = max(segments, key=lambda s: len(s[0]))
    if width is None:
        width = rng.uniform(2.8, 4.2)       # make_worlds.draw_course default

    centre, _ = resample_closed(pts, n_samples, closed=closed)
    centre = centre + MODEL_OFFSET
    widths = (np.full(n_samples, float(width)) if np.ndim(width) == 0
              else np.interp(np.linspace(0, 1, n_samples),
                             np.linspace(0, 1, len(np.asarray(width))),
                             np.asarray(width, dtype=np.float64)))

    barrels = mw.place_barrels([(np.asarray(pts), closed, width)], rng)
    discs = np.zeros((n_obstacles, 2))
    radii = np.zeros(n_obstacles)
    for i, (bx, by, _) in enumerate(barrels[:n_obstacles]):
        discs[i] = (bx + MODEL_OFFSET[0], by + MODEL_OFFSET[1])
        radii[i] = 0.25                      # worlds/sim_world_ic.sdf
    return _pack(name, centre, widths, closed, discs, radii)


def igvc_course(n_samples=N_SAMPLES, n_obstacles=N_OBSTACLES, config_dir=CONFIG):
    """The real course, from the two generated config files."""
    import yaml
    with open(os.path.join(config_dir, "course_meta.yaml")) as fh:
        meta = yaml.safe_load(fh)
    rows = np.array(meta["centreline"], dtype=np.float64)
    centre, _ = resample_closed(rows[:, :2], n_samples, closed=True)
    widths = np.interp(np.linspace(0, 1, n_samples),
                       np.linspace(0, 1, len(rows)), rows[:, 2])

    discs = np.zeros((n_obstacles, 2))
    radii = np.zeros(n_obstacles)
    obst_path = os.path.join(config_dir, "course_obstacles.yaml")
    if os.path.exists(obst_path):
        with open(obst_path) as fh:
            obst = yaml.safe_load(fh) or {}
        items = list(obst.get("barrels") or [])
        # Barricades are 3 x 0.5 x 3 boxes; as discs they get a radius that
        # covers the short axis, deliberately under-covering the long one so
        # the policy is not taught that they are wider than they are.
        for row in (obst.get("barricades") or []):
            items.append([row[0], row[1], max(0.5, row[4]) / 2.0])
        for i, row in enumerate(items[:n_obstacles]):
            discs[i] = (row[0], row[1])
            radii[i] = row[2]
    return _pack("igvc", centre, widths, True, discs, radii)


def _pack(name, centre, widths, closed, discs, radii):
    s, length = arclength(centre, closed=closed)
    left, right = boundaries(centre, widths, closed=closed)
    return {
        "name": name,
        "centre": centre,
        "widths": widths,
        "arclen": s,
        "length": length,
        "closed": bool(closed),
        "left": left,
        "right": right,
        "discs": discs,
        "radii": radii,
    }


# -------------------------------------------------------------------- bank --
def build_bank(names=DEFAULT_COURSES, per_course=4, seed=0, include_igvc=False,
               n_samples=N_SAMPLES, n_obstacles=N_OBSTACLES):
    """A stack of courses as rectangular arrays.

    include_igvc defaults to False on purpose: the real course is the
    EVALUATION course. Training on it and then reporting success on it would
    measure memorisation, and 34 barrel positions are very memorisable.
    """
    rng = random.Random(seed)
    courses = []
    for name in names:
        for _ in range(per_course):
            courses.append(generated_course(name, rng, n_samples, n_obstacles))
    if include_igvc:
        courses.append(igvc_course(n_samples, n_obstacles))
    if not courses:
        raise ValueError("empty course bank")

    stack = lambda key: np.stack([c[key] for c in courses])  # noqa: E731
    return {
        "names": [c["name"] for c in courses],
        "centre": stack("centre"),
        "widths": stack("widths"),
        "arclen": stack("arclen"),
        "length": np.array([c["length"] for c in courses]),
        "closed": np.array([c["closed"] for c in courses]),
        "left": stack("left"),
        "right": stack("right"),
        "discs": stack("discs"),
        "radii": stack("radii"),
    }


def summarise(bank):
    lines = []
    n = len(bank["names"])
    lines.append("%d courses, %d centreline samples, %d obstacle slots"
                 % (n, bank["centre"].shape[1], bank["discs"].shape[1]))
    lines.append("  lap length   min %.1f  median %.1f  max %.1f m"
                 % (bank["length"].min(), float(np.median(bank["length"])),
                    bank["length"].max()))
    w = bank["widths"]
    lines.append("  corridor     min %.2f  median %.2f  max %.2f m"
                 % (w.min(), float(np.median(w)), w.max()))
    live = (bank["radii"] > 0).sum(axis=1)
    lines.append("  obstacles    min %d  median %d  max %d"
                 % (live.min(), int(np.median(live)), live.max()))
    span = bank["centre"].reshape(-1, 2)
    lines.append("  extent       x %.1f..%.1f  y %.1f..%.1f"
                 % (span[:, 0].min(), span[:, 0].max(),
                    span[:, 1].min(), span[:, 1].max()))
    return "\n".join(lines)


if __name__ == "__main__":
    bank = build_bank(per_course=2, include_igvc=True)
    print(summarise(bank))
    print("  names:", ", ".join(sorted(set(bank["names"]))))
