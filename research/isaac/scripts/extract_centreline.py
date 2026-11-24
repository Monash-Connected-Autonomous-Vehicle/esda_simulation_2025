#!/usr/bin/env python3
"""Recover the drivable centreline of the IGVC course from its ground texture.

WHY THIS IS HARDER THAN IT LOOKS
    worlds/models/orange_igvc/model.sdf has no centreline anywhere.  The course
    is a 43 x 37 m quad whose lane markings live *in a texture* -- ground.png,
    1400 x 1200 -- and nothing else.  The RL reward needs an arc-length to make
    progress along, so that centreline has to come from somewhere.

    The obvious approach is to flood fill the drivable corridor and take its
    medial axis.  Measured on the real texture, that does not work: at every
    paint threshold the entire free space is ONE connected component covering
    100% of non-paint pixels.  The painted boundary has large gaps -- the
    top-centre ramp section is not joined to the side lanes at all -- so a fill
    seeded at the robot spawn leaks straight into the infield.  Dilating the
    paint to bridge the gaps fragments the corridor before it closes them:

        dilate k=0..2   one component, 100% reachable   (no corridor at all)
        dilate k=4..14  98-99% reachable                (still leaking)
        dilate k>=18    the corridor itself breaks up

WHAT WORKS INSTEAD
    The topology is already written down.  waypoints/igvc_waypoints.yaml holds
    four poses along the course, and scripts/ui_launch.py spawns the robot at
    world (11, 0).  Those five points say which way round the loop goes.  The
    geometry then comes from a least-cost route between them, over a cost field
    that prefers cells whose distance to the nearest paint is about half a lane
    width:

        cost(px) = 1 + alpha * |clearance(px) - half_width|

    In the corridor, clearance peaks at roughly half the lane width, so the
    route hugs the middle.  Out in the infield clearance is hundreds of pixels,
    so |clearance - half_width| is enormous and the route will not cross it --
    which is exactly the discrimination the flood fill could not make.  Cells
    closer to the paint than the robot radius are removed outright, so the
    result cannot scrape a line.

    The anchors are snapped to the nearest local clearance maximum first.  The
    waypoint file states its own accuracy as about 0.2 m for the active list and
    "1-2 m" for the competition lists, and measurement puts ramp_start about
    1.1 m off the corridor centre -- fine as topology, too coarse as geometry.

VALIDATION THAT DOES NOT BEG THE QUESTION
    The anchors cannot validate a path that was built to pass through them.  So
    this script checks something independent: it recovers the four barricade
    positions from the texture's dark strokes and compares them against the
    coordinates in model.sdf.  That tests the world <-> pixel mapping, which is
    what everything else rests on.  Measured agreement is about 0.3 m.

OUTPUT
    research/isaac/config/course_meta.yaml  centreline, width profile, obstacles
    research/isaac/usd/centreline_overlay.png  look at this before trusting it

USAGE
    python3 research/isaac/scripts/extract_centreline.py
    python3 research/isaac/scripts/extract_centreline.py --no-overlay
"""
import argparse
import heapq
import math
import os
import sys
import xml.dom.minidom as minidom

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
PKG = os.path.join(REPO, "src", "esda_simulation_2025")
MODEL_DIR = os.path.join(PKG, "worlds", "models", "orange_igvc")
GROUND_PNG = os.path.join(MODEL_DIR, "materials", "textures", "ground.png")
MODEL_SDF = os.path.join(MODEL_DIR, "model.sdf")
OUT_YAML = os.path.join(REPO, "research", "isaac", "config", "course_meta.yaml")
OUT_OVERLAY = os.path.join(REPO, "research", "isaac", "usd", "centreline_overlay.png")

# Arena constants, read off meshes/ground.dae and reused from
# research/vision/make_worlds.py so there is one copy of this geometry.
X_MIN, X_MAX = -21.5, 21.5
Y_MIN, Y_MAX = -18.5, 18.5
TEX_W, TEX_H = 1400, 1200
PX_PER_M_X = TEX_W / (X_MAX - X_MIN)      # 32.558
PX_PER_M_Y = TEX_H / (Y_MAX - Y_MIN)      # 32.432
PX_PER_M = 0.5 * (PX_PER_M_X + PX_PER_M_Y)

# igvc.sdf includes the orange_igvc model at pose (0, 16.3, 0).
MODEL_OFFSET = (0.0, 16.3)

PAINT_THRESHOLD = 200      # gray; paint is 242,242,242, asphalt is ~128
DARK_THRESHOLD = 60        # the barricade strokes
MIN_PAINT_AREA_PX = 100    # below this it is texture speckle, not a marking

# lane_half_width in scripts/waypoint_navigator_recommendation.py
HALF_WIDTH_M = 1.5
# robot_radius in config/nav2_params.yaml
ROBOT_RADIUS_M = 0.45

# worlds/sim_world_ic.sdf approximates construction_barrel.dae this way.
BARREL_RADIUS_M = 0.25
# Preferred standoff from a barrel surface. The hard limit is the robot radius;
# this is what the route is nudged toward, so the reference path does not skim
# obstacles at exactly the legal minimum.
OBSTACLE_PREFER_M = 0.80

# scripts/ui_launch.py spawns at world (11, 0); map frame origin is that pose.
SPAWN_WORLD = (11.0, 0.0)

# waypoints/igvc_waypoints.yaml, the active "waypoints:" list, in map metres.
# Order is the direction of travel: up the right side, right-to-left across the
# top through the ramp, then down the left side and along the bottom.
ROUTE_MAP_FRAME = [
    ("no_mans_land_start", (4.23, 30.04)),
    ("ramp_start", (-5.00, 30.70)),
    ("ramp_end", (-16.51, 30.71)),
    ("no_mans_land_end", (-26.37, 30.27)),
]


# -------------------------------------------------------------- geometry --
def world_to_px(x, y):
    """World metres (igvc.sdf frame) -> texture pixel (col, row)."""
    mx, my = x - MODEL_OFFSET[0], y - MODEL_OFFSET[1]
    col = (mx - X_MIN) / (X_MAX - X_MIN) * TEX_W
    row = (Y_MAX - my) / (Y_MAX - Y_MIN) * TEX_H
    return col, row


def px_to_world(col, row):
    mx = col / TEX_W * (X_MAX - X_MIN) + X_MIN
    my = Y_MAX - row / TEX_H * (Y_MAX - Y_MIN)
    return mx + MODEL_OFFSET[0], my + MODEL_OFFSET[1]


# ------------------------------------------------------------- the texture --
def load_masks(png):
    img = cv2.imread(png, cv2.IMREAD_COLOR)
    if img is None:
        sys.exit("cannot read %s" % png)
    if img.shape[:2] != (TEX_H, TEX_W):
        sys.exit("expected a %dx%d texture, got %dx%d -- the arena constants "
                 "in this file came from meshes/ground.dae and would be wrong"
                 % (TEX_W, TEX_H, img.shape[1], img.shape[0]))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    paint = despeckle((gray >= PAINT_THRESHOLD).astype(np.uint8), MIN_PAINT_AREA_PX)
    dark = (gray <= DARK_THRESHOLD).astype(np.uint8)
    return img, paint, dark


def despeckle(mask, min_area):
    """Drop connected components smaller than min_area.

    ground.png is sprinkled with single-pixel coloured specks -- texture noise,
    not markings -- and several are bright enough to pass the paint threshold.
    Left in, they wreck the corridor test: a lone speck 2 m north of a cell
    satisfies "paint on the opposite side" and invents a corridor out in the
    open, which is how the route first ended up running outside the ramp
    section's upper line.  The real lane components are all >= 400 px.
    """
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    keep = np.zeros(n, dtype=bool)
    for i in range(1, n):
        keep[i] = stats[i, cv2.CC_STAT_AREA] >= min_area
    return keep[lab].astype(np.uint8)


def _link_poses(path, prefix):
    """World-frame (x, y) of every link in model.sdf whose name starts with prefix."""
    doc = minidom.parse(path)
    out = {}
    for link in doc.getElementsByTagName("link"):
        name = link.getAttribute("name")
        if not name.startswith(prefix):
            continue
        for pose in link.childNodes:
            if pose.nodeType == pose.ELEMENT_NODE and pose.tagName == "pose":
                vals = [float(v) for v in pose.firstChild.data.split()]
                out[name] = (vals[0] + MODEL_OFFSET[0], vals[1] + MODEL_OFFSET[1])
                break
    return out


def barricades_from_sdf(path):
    """The four barricade poses declared in model.sdf, in world metres."""
    return _link_poses(path, "barricade")


def barrels_from_sdf(path):
    """The construction barrels, in world metres.

    Read from the SDF rather than from config/course_obstacles.yaml so this
    script has no ordering dependency on build_course_usd.py -- the centreline
    and the obstacle list are then both derived from the same source, and
    neither can be regenerated into disagreement with the other.
    """
    return sorted(_link_poses(path, "construction_barrel").values())


def barricades_from_texture(dark, min_area=50):
    n, lab, stats, cent = cv2.connectedComponentsWithStats(dark, 8)
    found = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_area:
            continue
        found.append(px_to_world(cent[i][0], cent[i][1]))
    return found


def check_mapping(dark, sdf_path):
    """Independent check of world <-> pixel, using geometry we did not fit."""
    declared = barricades_from_sdf(sdf_path)
    found = barricades_from_texture(dark)
    rows, worst = [], 0.0
    for name, (dx, dy) in sorted(declared.items()):
        if not found:
            break
        best = min(found, key=lambda p: (p[0] - dx) ** 2 + (p[1] - dy) ** 2)
        err = math.hypot(best[0] - dx, best[1] - dy)
        worst = max(worst, err)
        rows.append((name, (dx, dy), best, err))
    return rows, worst


# ---------------------------------------------------------------- routing --
def _shift(a, dr, dc):
    """a shifted so out[r, c] == a[r + dr, c + dc]; off-grid reads as 0."""
    out = np.zeros_like(a)
    h, w = a.shape
    src_r = slice(max(0, dr), min(h, h + dr))
    src_c = slice(max(0, dc), min(w, w + dc))
    dst_r = slice(max(0, -dr), min(h, h - dr))
    dst_c = slice(max(0, -dc), min(w, w - dc))
    out[dst_r, dst_c] = a[src_r, src_c]
    return out


# 8 compass directions; d and d+4 are opposites.
_DIRS = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]


def corridor_mask(paint, reach_m=2.5, step_px=2):
    """True where paint lies within reach in two OPPOSING directions.

    This is the test the clearance field cannot make.  A cell 1.5 m outside a
    single painted line has exactly the same distance-to-paint as a cell in the
    middle of a 3 m corridor, so a cost built on clearance alone is equally
    happy on either -- and on the real texture it picks the wrong one, running
    the route over the top of the ramp section's upper line instead of between
    the two lines.  Requiring paint on both sides removes that whole family of
    false corridors, because outside a line there is nothing further out.

    It also, correctly, reports no corridor across no man's land: the IGVC
    course genuinely has unmarked gaps there (hence the waypoint names
    no_mans_land_start / no_mans_land_end), and the robot crosses them on GPS.
    """
    reach_px = reach_m * PX_PER_M
    hits = []
    for dr, dc in _DIRS:
        norm = math.hypot(dr, dc)
        steps = max(1, int(reach_px / (step_px * norm)))
        acc = np.zeros_like(paint, dtype=bool)
        for s in range(1, steps + 1):
            acc |= _shift(paint, dr * s * step_px, dc * s * step_px).astype(bool)
        hits.append(acc)
    mask = np.zeros_like(paint, dtype=bool)
    for d in range(4):
        mask |= hits[d] & hits[d + 4]
    return mask & (paint == 0)


def obstacle_field(barrels, barrel_radius=BARREL_RADIUS_M):
    """Distance in metres from every pixel to the nearest barrel SURFACE.

    WHY THE CENTRELINE HAS TO KNOW ABOUT THESE
        The first version of this script routed on lane paint alone. Barrels
        are 3D models, not painted, so they were invisible to it -- and the
        resulting path ran 0.11 m from barrel 4 and straight between three
        barrel pairs whose gaps are 0.62-0.65 m when the robot needs 0.90 m.

        That is not a small inaccuracy. The RL reward pays the policy to follow
        this line, so an obstacle-blind centreline teaches the robot to drive
        into barrels, and it did: 84% of collisions in evaluation were three
        barrels sitting on or beside this path.
    """
    free = np.ones((TEX_H, TEX_W), dtype=np.uint8)
    for bx, by in barrels:
        col, row = world_to_px(bx, by)
        cv2.circle(free, (int(round(col)), int(round(row))),
                   max(1, int(round(barrel_radius * PX_PER_M))), 0, -1)
    if free.all():
        return np.full((TEX_H, TEX_W), 1e3, dtype=np.float32)
    return (cv2.distanceTransform(free, cv2.DIST_L2, 5) / PX_PER_M).astype(np.float32)


def build_cost(paint, barrels=(), nomans_cost=6.0, obstacle_weight=3.0):
    """Per-pixel traversal cost, the clearance field, and the blocked band.

    Inside a corridor the cost rewards clearance near half a lane width, so the
    route hugs the middle.  Where there is no corridor -- no man's land -- the
    cost is flat and higher, so the route crosses in roughly a straight line and
    prefers a real corridor whenever one exists.

    Barrels block outright within the robot's radius and are penalised softly
    out to OBSTACLE_PREFER_M, so the route gives them room rather than merely
    squeezing past. Blocking alone would produce paths that skim an obstacle at
    exactly the legal minimum, which is not what you want a policy to imitate.
    """
    clearance_px = cv2.distanceTransform(1 - paint, cv2.DIST_L2, 5)
    clearance_m = (clearance_px / PX_PER_M).astype(np.float32)
    corridor = corridor_mask(paint)
    cost = np.where(corridor,
                    1.0 + 4.0 * np.abs(clearance_m - HALF_WIDTH_M),
                    nomans_cost).astype(np.float32)
    blocked = clearance_m < ROBOT_RADIUS_M

    obstacle_m = None
    if len(barrels):
        obstacle_m = obstacle_field(barrels)
        blocked = blocked | (obstacle_m < ROBOT_RADIUS_M)
        near = np.clip(OBSTACLE_PREFER_M - obstacle_m, 0.0, None)
        cost = cost + obstacle_weight * near
    return cost, clearance_m, blocked, corridor, obstacle_m


def snap_anchor(clearance, blocked, corridor, col, row, radius_px=70):
    """Move an anchor onto the corridor centre nearby, if there is a corridor.

    The waypoint file rates itself at about 0.2 m for the active list and 1-2 m
    for the competition lists; ramp_start measures about 1.1 m off the corridor
    centre.  Good enough to say which way round the loop goes, not good enough
    to be a path vertex.  Corridor cells are always preferred over non-corridor
    ones, so an anchor beside the ramp section snaps between its two lines
    rather than outside them.
    """
    c0, r0 = int(round(col)), int(round(row))
    lo_r, hi_r = max(0, r0 - radius_px), min(TEX_H, r0 + radius_px + 1)
    lo_c, hi_c = max(0, c0 - radius_px), min(TEX_W, c0 + radius_px + 1)
    clr = clearance[lo_r:hi_r, lo_c:hi_c]
    blk = blocked[lo_r:hi_r, lo_c:hi_c]
    cor = corridor[lo_r:hi_r, lo_c:hi_c]

    score = np.abs(clr - HALF_WIDTH_M).astype(np.float64)
    score[~cor] += 1000.0          # corridor cells win outright
    score[blk] = np.inf
    if not np.isfinite(score).any():
        return c0, r0, float(clearance[r0, c0]), False
    idx = int(np.argmin(score))
    rr, cc = divmod(idx, clr.shape[1])
    return lo_c + cc, lo_r + rr, float(clr[rr, cc]), bool(cor[rr, cc])


_NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
               (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def shortest_path(cost, blocked, start, goal):
    """8-connected Dijkstra from start to goal over `cost`. Pixels are (col,row).

    A* with a Euclidean heuristic: the cost floor is 1.0 per unit step, so plain
    straight-line distance is admissible and keeps the expansion local instead
    of flooding all 1.68 M cells.
    """
    h, w = cost.shape
    sc, sr = start
    gc, gr = goal
    if blocked[sr, sc] or blocked[gr, gc]:
        raise ValueError("endpoint is inside the blocked band")

    INF = np.float32(np.inf)
    dist = np.full((h, w), INF, dtype=np.float32)
    prev = np.full((h, w), -1, dtype=np.int64)
    dist[sr, sc] = 0.0
    heap = [(0.0, sr * w + sc)]
    visited = np.zeros((h, w), dtype=bool)

    while heap:
        _, node = heapq.heappop(heap)
        r, c = divmod(node, w)
        if visited[r, c]:
            continue
        visited[r, c] = True
        if r == gr and c == gc:
            break
        d = dist[r, c]
        for dr, dc, step in _NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= h or nc < 0 or nc >= w or blocked[nr, nc] or visited[nr, nc]:
                continue
            nd = d + step * cost[nr, nc]
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                prev[nr, nc] = node
                heapq.heappush(heap, (nd + math.hypot(nr - gr, nc - gc), nr * w + nc))

    if not visited[gr, gc]:
        raise ValueError("no route from %s to %s" % (start, goal))

    path, node = [], gr * w + gc
    while node != -1:
        r, c = divmod(node, w)
        path.append((c, r))
        node = prev[r, c]
    path.reverse()
    return path


# ------------------------------------------------------------ postprocess --
def resample(points_m, step_m):
    """Even arc-length resampling of a closed polyline."""
    pts = np.asarray(points_m, dtype=np.float64)
    closed = np.vstack([pts, pts[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    want = np.arange(0.0, total, step_m)
    out = np.stack([np.interp(want, s, closed[:, 0]),
                    np.interp(want, s, closed[:, 1])], axis=1)
    return out, total


def smooth_closed(pts, k):
    """Box filter with wraparound -- the course is a loop."""
    if k < 3:
        return pts
    pad = k // 2
    ext = np.vstack([pts[-pad:], pts, pts[:pad]])
    ker = np.ones(k) / k
    sx = np.convolve(ext[:, 0], ker, "valid")
    sy = np.convolve(ext[:, 1], ker, "valid")
    return np.stack([sx, sy], axis=1)


def push_clear_of_obstacles(pts, barrels, want, iterations=6):
    """Nudge a smoothed path back out of obstacles.

    Routing already refuses cells within the robot radius of a barrel, but the
    box filter that follows cuts corners -- measured, it pulled the path to
    0.30 m from barrel 26 when the robot needs 0.45 m. Smoothing a path that
    satisfies a clearance constraint does not preserve it.

    Each pass moves offending samples radially outward to `want` and then
    lightly re-smooths, so the result stays a smooth curve rather than
    developing a kink at every repair.
    """
    if not len(barrels):
        return pts, 0
    obs = np.asarray(barrels, dtype=np.float64)
    pts = pts.copy()
    moved_total = 0
    for _ in range(iterations):
        delta = pts[:, None, :] - obs[None, :, :]
        dist = np.linalg.norm(delta, axis=2) - BARREL_RADIUS_M
        j = dist.argmin(axis=1)
        nearest = dist[np.arange(len(pts)), j]
        bad = nearest < want
        if not bad.any():
            break
        moved_total += int(bad.sum())
        vec = pts[bad] - obs[j[bad]]
        norm = np.linalg.norm(vec, axis=1, keepdims=True)
        vec = vec / np.where(norm < 1e-9, 1.0, norm)
        pts[bad] = obs[j[bad]] + vec * (BARREL_RADIUS_M + want)
        pts = smooth_closed(pts, 5)
    return pts, moved_total


def obstacle_clearance(pts, barrels):
    """Closest approach of the path to any barrel surface, and where."""
    if not len(barrels):
        return float("inf"), -1
    obs = np.asarray(barrels, dtype=np.float64)
    dist = np.linalg.norm(pts[:, None, :] - obs[None, :, :], axis=2) - BARREL_RADIUS_M
    per = dist.min(axis=0)
    i = int(per.argmin())
    return float(per[i]), i


def sample_clearance(clearance, pts_m):
    cols = np.clip(((pts_m[:, 0] - MODEL_OFFSET[0] - X_MIN) / (X_MAX - X_MIN) * TEX_W),
                   0, TEX_W - 1).astype(int)
    rows = np.clip(((Y_MAX - (pts_m[:, 1] - MODEL_OFFSET[1])) / (Y_MAX - Y_MIN) * TEX_H),
                   0, TEX_H - 1).astype(int)
    return clearance[rows, cols]


# ------------------------------------------------------------------- main --
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--png", default=GROUND_PNG)
    ap.add_argument("--sdf", default=MODEL_SDF)
    ap.add_argument("--out", default=OUT_YAML)
    ap.add_argument("--overlay", default=OUT_OVERLAY)
    ap.add_argument("--no-overlay", action="store_true")
    ap.add_argument("--step", type=float, default=0.25,
                    help="centreline resampling step in metres")
    ap.add_argument("--smooth", type=int, default=15,
                    help="box-filter width in samples; 0 disables")
    args = ap.parse_args(argv)

    img, paint, dark = load_masks(args.png)
    print("paint: %d px (%.2f%%)   dark: %d px" % (paint.sum(), 100.0 * paint.mean(), dark.sum()))

    rows, worst = check_mapping(dark, args.sdf)
    print("\nworld<->pixel check, barricades in model.sdf vs dark strokes in the texture:")
    for name, declared, found, err in rows:
        print("  %-12s sdf (%+7.2f,%+7.2f)  texture (%+7.2f,%+7.2f)  err %.2f m"
              % (name, declared[0], declared[1], found[0], found[1], err))
    print("  worst %.2f m" % worst)
    if worst > 1.0:
        print("  WARNING: the world<->pixel mapping looks wrong. Everything "
              "downstream inherits this error.")

    barrels = barrels_from_sdf(args.sdf)
    cost, clearance, blocked, corridor, obstacle_m = build_cost(paint, barrels)
    print("\nbarrels from model.sdf: %d, blocked within %.2f m of a surface, "
          "preferred standoff %.2f m" % (len(barrels), ROBOT_RADIUS_M,
                                         OBSTACLE_PREFER_M))
    print("\nclearance: max %.2f m, blocked (<%.2f m) %.1f%% of texture"
          % (clearance.max(), ROBOT_RADIUS_M, 100.0 * blocked.mean()))
    print("corridor (paint within 2.5 m on two opposing sides): %.1f%% of texture"
          % (100.0 * corridor.mean()))

    anchors = [("spawn", SPAWN_WORLD)]
    anchors += [(name, (mx + SPAWN_WORLD[0], my + SPAWN_WORLD[1]))
                for name, (mx, my) in ROUTE_MAP_FRAME]

    print("\nanchors (map frame -> world -> snapped):")
    snapped = []
    for name, (wx, wy) in anchors:
        col, row = world_to_px(wx, wy)
        sc, sr, clr, in_cor = snap_anchor(clearance, blocked, corridor, col, row)
        swx, swy = px_to_world(sc, sr)
        moved = math.hypot(swx - wx, swy - wy)
        print("  %-20s world (%+7.2f,%+7.2f) -> (%+7.2f,%+7.2f)  moved %.2f m, "
              "clearance %.2f m, %s" % (name, wx, wy, swx, swy, moved, clr,
                                        "corridor" if in_cor else "NO MAN'S LAND"))
        snapped.append((sc, sr))

    print("\nrouting %d legs..." % len(snapped))
    path_px = []
    for i in range(len(snapped)):
        a, b = snapped[i], snapped[(i + 1) % len(snapped)]
        leg = shortest_path(cost, blocked, a, b)
        print("  leg %d: %s -> %s   %d px" % (i, a, b, len(leg)))
        path_px.extend(leg if i == 0 else leg[1:])

    pts_m = np.array([px_to_world(c, r) for c, r in path_px], dtype=np.float64)
    pts_m, raw_len = resample(pts_m, args.step)
    pts_m = smooth_closed(pts_m, args.smooth)
    pts_m, repaired = push_clear_of_obstacles(pts_m, barrels, OBSTACLE_PREFER_M)
    if repaired:
        print("  repaired %d smoothed samples that had drifted inside the "
              "obstacle standoff" % repaired)
    widths = 2.0 * sample_clearance(clearance, pts_m)

    _, length = resample(pts_m, args.step)
    print("\ncentreline: %d samples, %.1f m lap (raw %.1f m)" % (len(pts_m), length, raw_len))
    print("  corridor width: min %.2f  median %.2f  max %.2f m"
          % (widths.min(), float(np.median(widths)), widths.max()))
    print("  clearance:      min %.2f m  (robot radius %.2f)"
          % (widths.min() / 2.0, ROBOT_RADIUS_M))
    if widths.min() / 2.0 < ROBOT_RADIUS_M:
        print("  WARNING: the route passes closer to the paint than the robot "
              "radius. Inspect the overlay.")

    gap, which = obstacle_clearance(pts_m, barrels)
    print("  barrel clearance: closest approach %.2f m (barrel %d), "
          "robot radius %.2f" % (gap, which, ROBOT_RADIUS_M))
    if gap < ROBOT_RADIUS_M:
        print("  WARNING: the reference path passes closer to a barrel than the")
        print("           robot radius. A policy rewarded for following it will")
        print("           be rewarded for driving into that barrel.")

    write_yaml(args.out, pts_m, widths, length, worst)
    print("\nwrote %s" % args.out)

    if not args.no_overlay:
        draw_overlay(args.overlay, img, pts_m, snapped)
        print("wrote %s  -- LOOK AT THIS before trusting the centreline" % args.overlay)
    return 0


def write_yaml(path, pts, widths, length, mapping_err):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("# GENERATED by research/isaac/scripts/extract_centreline.py\n")
        fh.write("# Do not edit by hand -- regenerate instead.\n")
        fh.write("#\n")
        fh.write("# The drivable centreline of worlds/igvc.sdf, recovered from the\n")
        fh.write("# lane paint in ground.png by routing between the poses in\n")
        fh.write("# waypoints/igvc_waypoints.yaml. See that script for why the\n")
        fh.write("# obvious flood-fill approach does not work on this texture.\n")
        fh.write("#\n")
        fh.write("# Frame: igvc.sdf world metres (the orange_igvc model is included\n")
        fh.write("# at pose 0 16.3 0, and that offset is already applied here), so\n")
        fh.write("# these coordinates line up with the Gazebo sim, my_map_save.pgm\n")
        fh.write("# and the Isaac course USD.\n")
        fh.write("name: igvc\n")
        fh.write("source: worlds/igvc.sdf\n")
        fh.write("closed: true\n")
        fh.write("lap_length_m: %.3f\n" % length)
        fh.write("nominal_half_width_m: %.3f\n" % HALF_WIDTH_M)
        fh.write("world_pixel_check_worst_err_m: %.3f\n" % mapping_err)
        fh.write("start_pose: {x: %.3f, y: %.3f, yaw: 0.0}\n" % SPAWN_WORLD)
        fh.write("# x, y, corridor_width -- one row per sample, evenly spaced\n")
        fh.write("centreline:\n")
        for (x, y), w in zip(pts, widths):
            fh.write("  - [%.4f, %.4f, %.4f]\n" % (x, y, w))


def draw_overlay(path, img, pts, anchors_px):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    out = img.copy()
    poly = np.array([world_to_px(x, y) for x, y in pts], dtype=np.int32)
    cv2.polylines(out, [poly], True, (0, 0, 255), 3)
    for i, (c, r) in enumerate(anchors_px):
        cv2.circle(out, (int(c), int(r)), 14, (255, 0, 0), 3)
        cv2.putText(out, str(i), (int(c) + 18, int(r) + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
    cv2.imwrite(path, out)


if __name__ == "__main__":
    sys.exit(main())
