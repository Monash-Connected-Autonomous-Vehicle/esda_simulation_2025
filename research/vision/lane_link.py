#!/usr/bin/env python3
"""
ESDA lane linking — join discontinuous lane fragments into whole lane lines.

THE PROBLEM
    A lane boundary reaches the costmap as a string of disconnected blobs:
      * the marking is dashed,
      * a barrel stands on it and hides a chunk,
      * the paint is worn, or the model simply misses the far end.
    Nav2 does not know those blobs are one wall. It happily plans a path
    straight through a dash gap. Marking cells is not enough -- the boundary
    has to be CONTINUOUS.

THE APPROACH
    Everything happens in GROUND space (base_link metres), never in pixels.
    Perspective makes image distance meaningless: a 2 m dash gap at 10 m is a
    couple of pixels, while the same gap at 1.5 m is half the frame. On the
    ground a gap is just a gap.

    1. rasterise the projected lane points into a bird's-eye grid
    2. connected components -> fragments
    3. seed a lane line from the longest unclaimed fragment
    4. GROW it: fit a curve to the group so far, then absorb any fragment
       whose position lies close to that curve (extrapolated a little past
       the ends). Refit and repeat until nothing more is absorbed.
    5. resample the final curve densely -- this is what closes the gaps.

    WHY GROWING, NOT PAIRWISE ENDPOINT MATCHING
    The obvious design -- join fragment endpoints whose headings agree -- was
    tried first and measured at 0.17 recall. The reason is in the data: real
    detected fragments have a MEDIAN LENGTH OF 0.50 m, and a stubby 0.5 m by
    0.12 m blob has no meaningful principal axis. Same-line fragment pairs
    disagreed in heading by a median of 52 degrees. Per-fragment direction is
    simply not a usable signal here.
    A short fragment's POSITION, though, is perfectly reliable. So direction
    comes only from the accumulated group curve, which has real extent, and
    each fragment is judged purely on how far it sits from that curve.

    Extrapolation is deliberately throttled: a group may only reach past its
    own ends by about its own current span (clamped into
    [min_extrap, max_gap]). A one-fragment seed therefore probes only ~0.8 m,
    and the reach grows as the evidence does, instead of a noisy two-point
    axis firing off across the whole frame.

    Separating the two boundaries is what the lateral tolerance buys. The
    opposite lane line sits a corridor-width away (2.4-4.4 m here), five-plus
    times the ~0.5 m tolerance, so it can never be absorbed.

No ROS and no sklearn: numpy + cv2 only, so it is unit-testable on its own.
"""
import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


DEFAULTS = {
    "cell": 0.05,          # bird's-eye grid resolution, metres
    "x_range": [0.3, 9.0],  # forward extent considered, metres
    "y_range": [-5.0, 5.0],
    "close_cells": 2,      # morphological close, in CELLS. Keep small: this is
                           # meant to heal pixel speckle, NOT to jump dash gaps.
    "min_fragment_cells": 4,
    "max_gap": 2.5,        # furthest a group may reach past its own ends, m.
                           # Was 3.5. The real dash gap is 1.65 m, so 2.5
                           # still joins dashes with margin while halving how
                           # much line can be invented in one go.
    "min_extrap": 1.8,     # ...but a tiny seed may only reach this far.
                           # Must exceed the real dash GAP, which here is
                           # period*(1-duty) = up to 3.0*0.55 = 1.65 m. Set it
                           # to 0.8 and dashes never join.
    "max_lateral": 0.50,   # distance from the fitted curve to absorb, metres
    "grow_iters": 6,
    "fit_degree": 2,
    "resample_step": 0.025,  # spacing of emitted points, m. Must be below the
                            # costmap resolution or the raster skips cells.
    "max_fit_rms": 0.30,   # above this the chain is a bad merge -> don't bridge
    "max_bridge_bow": 0.25,  # metres a bridge may bow off the straight chord
                           # between the two fragments it joins.
                           #
                           # max_fit_rms is measured on the POINTS, so it says
                           # nothing about the span between them: a quadratic
                           # can fit two distant clusters perfectly and still
                           # bow a metre off course in the middle, where there
                           # is no evidence at all. Those invented points go
                           # into the costmap as obstacles, and since the lane
                           # layer no longer clears, a bad bridge is permanent.
                           # A dash gap is nearly collinear and bows by
                           # centimetres; a bad merge bows by a lot.
    "end_frac": 0.35,      # fraction of a fragment used for its local heading
}


class Fragment:
    __slots__ = ("idx", "pts", "centroid", "axis", "t", "a", "b",
                 "dir_a", "dir_b", "length")

    def __init__(self, idx, pts, end_frac):
        self.idx = idx
        self.pts = pts
        self.centroid = pts.mean(0)
        q = pts - self.centroid
        # principal axis
        if len(pts) >= 2:
            _, _, vt = np.linalg.svd(q, full_matrices=False)
            self.axis = vt[0] / (np.linalg.norm(vt[0]) + 1e-12)
        else:
            self.axis = np.array([1.0, 0.0])
        self.t = q @ self.axis
        self.a = pts[int(np.argmin(self.t))]      # one end
        self.b = pts[int(np.argmax(self.t))]      # the other end
        self.length = float(self.t.max() - self.t.min())
        self.dir_a = self._local_dir(self.t.min(), +1, end_frac)   # points OUT of a
        self.dir_b = self._local_dir(self.t.max(), -1, end_frac)   # points OUT of b

    def _local_dir(self, t_end, inward, end_frac):
        """Heading at one end, from the nearest slice of the fragment only.

        Using the whole fragment's axis would straighten out curves and make
        a curving lane fail the heading test at every joint.
        """
        span = max(self.length, 1e-6)
        win = max(span * end_frac, 0.12)
        sel = (np.abs(self.t - t_end) <= win)
        if sel.sum() < 2:
            d = self.axis.copy()
        else:
            p = self.pts[sel]
            q = p - p.mean(0)
            _, _, vt = np.linalg.svd(q, full_matrices=False)
            d = vt[0]
        # orient so it points away from the fragment body
        ref = self.axis * (-inward)
        if d @ ref < 0:
            d = -d
        return d / (np.linalg.norm(d) + 1e-12)

    def end(self, which):
        return (self.a, self.dir_a) if which == 0 else (self.b, self.dir_b)


def _grid_fragments(pts, cfg):
    """Rasterise to a bird's-eye grid and pull out connected components."""
    cell = cfg["cell"]
    x0, x1 = cfg["x_range"]
    y0, y1 = cfg["y_range"]
    if len(pts) == 0:
        return []
    m = ((pts[:, 0] >= x0) & (pts[:, 0] < x1) &
         (pts[:, 1] >= y0) & (pts[:, 1] < y1))
    pts = pts[m]
    if len(pts) == 0:
        return []

    nx = int(np.ceil((x1 - x0) / cell))
    ny = int(np.ceil((y1 - y0) / cell))
    gi = ((pts[:, 0] - x0) / cell).astype(int).clip(0, nx - 1)
    gj = ((pts[:, 1] - y0) / cell).astype(int).clip(0, ny - 1)
    grid = np.zeros((nx, ny), np.uint8)
    grid[gi, gj] = 1

    k = int(cfg["close_cells"])
    if k > 0 and cv2 is not None:
        grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE,
                                np.ones((2 * k + 1, 2 * k + 1), np.uint8))

    if cv2 is None:
        raise RuntimeError("lane_link needs cv2")
    n, lab, stats, _ = cv2.connectedComponentsWithStats(grid, 8)

    frags = []
    for c in range(1, n):
        if stats[c, cv2.CC_STAT_AREA] < cfg["min_fragment_cells"]:
            continue
        ii, jj = np.where(lab == c)
        p = np.stack([x0 + (ii + 0.5) * cell, y0 + (jj + 0.5) * cell], 1)
        frags.append(Fragment(len(frags), p, cfg["end_frac"]))
    return frags


class Curve:
    """A low-order curve fitted to a point set, in its own PCA frame.

    Outside the span of the data it continues as a straight tangent rather
    than letting a quadratic bend away, which is what keeps extrapolation
    across a dash gap sane.
    """

    def __init__(self, pts, degree):
        self.c = pts.mean(0)
        q = pts - self.c
        if len(pts) >= 2:
            _, _, vt = np.linalg.svd(q, full_matrices=False)
            self.e1 = vt[0] / (np.linalg.norm(vt[0]) + 1e-12)
        else:
            self.e1 = np.array([1.0, 0.0])
        self.e2 = np.array([-self.e1[1], self.e1[0]])
        s_ = q @ self.e1
        d_ = q @ self.e2
        self.s_min, self.s_max = float(s_.min()), float(s_.max())
        self.span = self.s_max - self.s_min
        deg = degree if (self.span > 1.5 and len(pts) >= 12) else 1
        deg = max(1, min(deg, len(pts) - 1)) if len(pts) > 1 else 0
        self.coef = np.polyfit(s_, d_, deg) if deg >= 1 else np.array([0.0, float(d_.mean())])
        self.rms = float(np.sqrt(np.mean((np.polyval(self.coef, s_) - d_) ** 2)))

    def _d_at(self, s):
        """Polynomial inside the data span, straight tangent outside it."""
        s = np.asarray(s, float)
        out = np.polyval(self.coef, np.clip(s, self.s_min, self.s_max))
        der = np.polyder(self.coef) if len(self.coef) > 1 else np.array([0.0])
        lo = s < self.s_min
        hi = s > self.s_max
        if lo.any():
            out[lo] += np.polyval(der, self.s_min) * (s[lo] - self.s_min)
        if hi.any():
            out[hi] += np.polyval(der, self.s_max) * (s[hi] - self.s_max)
        return out

    def project(self, pts):
        """Return (along-track s, signed lateral offset from the curve)."""
        q = np.asarray(pts, float).reshape(-1, 2) - self.c
        s = q @ self.e1
        d = q @ self.e2
        return s, d - self._d_at(s)

    def sample(self, s_lo, s_hi, step):
        ss = np.arange(s_lo, s_hi + step, step)
        return self.c + np.outer(ss, self.e1) + np.outer(self._d_at(ss), self.e2)


def _grow_groups(frags, cfg):
    """Seed on the longest unclaimed fragment, then absorb by position."""
    n = len(frags)
    unclaimed = set(range(n))
    order = sorted(range(n), key=lambda i: -frags[i].length)
    groups, curves = [], []

    for seed in order:
        if seed not in unclaimed:
            continue
        group = [seed]
        unclaimed.discard(seed)
        pts = frags[seed].pts
        curve = Curve(pts, cfg["fit_degree"])

        for _ in range(int(cfg["grow_iters"])):
            # reach grows with the evidence: a stubby seed barely extrapolates
            reach = float(np.clip(max(curve.span, cfg["min_extrap"]),
                                  cfg["min_extrap"], cfg["max_gap"]))
            added = []
            for j in list(unclaimed):
                s_j, d_j = curve.project(frags[j].pts)
                if float(np.abs(d_j).mean()) > cfg["max_lateral"]:
                    continue
                # must sit within the group's extent, plus the allowed reach
                if (s_j.max() < curve.s_min - reach) or (s_j.min() > curve.s_max + reach):
                    continue
                added.append(j)
            if not added:
                break
            for j in added:
                unclaimed.discard(j)
                group.append(j)
            pts = np.concatenate([frags[i].pts for i in group], 0)
            curve = Curve(pts, cfg["fit_degree"])

        groups.append(group)
        curves.append(curve)
    return groups, curves


def bridge_bows(curve, poly, ss, s_measured, gap_min):
    """Every stretch of curve with no measured points under it, and how far
    the fit bows off the straight chord joining the fragments either side.

    Separate from link_lanes() so it can be tested on a curve of known shape:
    this is the only thing standing between a bad fit and a permanent wall in
    a costmap that no longer clears.

    Returns a list of (s_lo, s_hi, bow_metres, mask_into_poly).
    """
    out = []
    s_sorted = np.sort(np.asarray(s_measured, float))
    for i in np.where(np.diff(s_sorted) > gap_min)[0]:
        a, b = float(s_sorted[i]), float(s_sorted[i + 1])
        inside = (ss > a) & (ss < b)
        if not inside.any():
            continue
        pa = curve.sample(a, a, 1.0)[0]
        pb = curve.sample(b, b, 1.0)[0]
        v = pb - pa
        L = float(np.linalg.norm(v))
        if L < 1e-9:
            continue
        nrm = np.array([-v[1], v[0]]) / L
        out.append((a, b, float(np.abs((poly[inside] - pa) @ nrm).max()), inside))
    return out


def bridge_drop_mask(curve, poly, ss, s_measured, gap_min, max_bow):
    """Mask of polyline samples sitting in a span that bows too far."""
    drop = np.zeros(len(poly), bool)
    nrej = 0
    for _a, _b, bow, inside in bridge_bows(curve, poly, ss, s_measured, gap_min):
        if bow > max_bow:
            drop |= inside
            nrej += 1
    return drop, nrej


class LaneLinkResult:
    def __init__(self):
        self.fragments = []       # Fragment objects, in detection order
        self.groups = []          # list of lists of fragment indices
        self.curves = []          # fitted Curve per group
        self.polylines = []       # dense Nx2 polyline per group (or None)
        self.group_of_fragment = {}
        self.dense = np.empty((0, 2), np.float32)   # all polylines concatenated
        self.bridged = 0          # how many fragment joins were made
        self.rejected_bridges = 0  # spans dropped for bowing off the chord


def link_lanes(points_xy, cfg=None):
    """points_xy: Nx2 ground points (X fwd, Y left) in base_link metres.

    Returns a LaneLinkResult whose `.dense` is a gap-free set of points along
    each reconstructed lane line -- that is what should go to the costmap.
    """
    c = dict(DEFAULTS)
    if cfg:
        c.update({k: v for k, v in cfg.items() if k in DEFAULTS})

    res = LaneLinkResult()
    pts = np.asarray(points_xy, np.float64).reshape(-1, 2)
    frags = _grid_fragments(pts, c)
    res.fragments = frags
    if not frags:
        return res

    groups, curves = _grow_groups(frags, c)
    res.groups, res.curves = groups, curves

    dense = []
    for gid, (g, curve) in enumerate(zip(groups, curves)):
        for fi in g:
            res.group_of_fragment[fi] = gid
        gp = np.concatenate([frags[i].pts for i in g], 0)

        # A high residual means the growth absorbed something it shouldn't
        # have. Fall back to the measured points and bridge nothing, rather
        # than drawing a confident wrong wall into the costmap.
        if curve.rms > c["max_fit_rms"]:
            res.polylines.append(None)
            dense.append(gp)
            continue

        s_, _ = curve.project(gp)
        s_lo, s_hi = float(s_.min()), float(s_.max())
        step = c["resample_step"]
        poly = curve.sample(s_lo, s_hi, step)
        ss = np.arange(s_lo, s_hi + step, step)       # must match sample()

        # Check every stretch of curve that has no measured points under it.
        # If the fit bows away from the straight chord joining the fragments
        # on either side, drop that stretch rather than paint it into the
        # costmap. The rest of the line is still emitted.
        drop, nrej = bridge_drop_mask(
            curve, poly, ss, s_, max(4.0 * c["cell"], 0.2), c["max_bridge_bow"])
        res.rejected_bridges += nrej
        if drop.any():
            poly = poly[~drop]

        res.polylines.append(poly)
        dense.append(poly)
        if len(g) > 1:
            res.bridged += len(g) - 1

    res.dense = (np.concatenate(dense, 0).astype(np.float32)
                 if dense else np.empty((0, 2), np.float32))
    return res
