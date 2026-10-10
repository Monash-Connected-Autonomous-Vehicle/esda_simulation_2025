#!/usr/bin/env python3
"""
Generate the `esda_carpark` Gazebo model: an IGVC-style parking lot that
worlds/igvc_carpark.sdf places around the unchanged orange_igvc track.

Modelled on the competition site: open asphalt with yellow stall lines,
light poles on concrete bases, a few parked cars, team tents, a black
picket fence around the lot, then grass and trees.

Stdlib only - no ROS or Gazebo needed.

    python3 tools/generate_carpark_world.py

Writes src/esda_simulation_2025/worlds/models/esda_carpark/{model.sdf,
model.config, meshes/fence.obj}. Edit the layout tables below and re-run; the
output is committed alongside this script.

All coordinates are WORLD frame. orange_igvc is included at (0, 16.3), so the
track occupies x in [-21.5, 21.5], y in [-2.2, 34.8] and the robot still spawns
at (11, 0).

What the lidar sees, compared with the other worlds: igvc.sdf has nothing
around the track, so most flat-scan rays return inf; igvc_campus.sdf has a
solid wall all round, so every ray returns. Here the fence pickets, tree
trunks, poles, cars and tent legs return some rays and leave real gaps, as on
the real site.

The generator refuses to write if the layout breaks any of:
  * nothing solid inside the track keep-out (track + KEEPOUT_MARGIN);
  * objects must not overlap each other, and must sit inside their zone
    (inside the fence for lot objects, outside it for trees);
  * every colour must stay at or below MAX_GRAY, except ELEVATED colours
    that are only used 2 m or more above the ground (tent roofs).
    lane_detection.py thresholds grayscale at 130, so anything brighter on
    or near the ground would be picked up as a lane line.
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_campus_world import (  # noqa: E402
    box_poly, circle_poly, gray, point_rect_dist, polys_overlap, rect_poly)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, 'src', 'esda_simulation_2025', 'worlds',
                         'models', 'esda_carpark')

# ---- geometry --------------------------------------------------------------

TRACK = (-21.5, 21.5, -2.2, 34.8)           # (xmin, xmax, ymin, ymax)
KEEPOUT_MARGIN = 3.0                        # nothing solid this close to the track
LOT = (-36.0, 36.0, -16.0, 50.0)            # asphalt, fenced
WORLD = (-60.0, 60.0, -40.0, 74.0)          # grass out to here
OBJECT_GAP = 0.5                            # min gap between footprints

# Black picket fence around the lot, like the competition photos. The gate
# gap on the south side is where the access road would be.
FENCE_HEIGHT = 1.5
FENCE_PICKET = 0.02                         # square picket width
FENCE_SPACING = 0.13                        # picket centre-to-centre
FENCE_POST = 0.06
FENCE_POST_SPACING = 2.4
FENCE_RAIL = 0.04                           # rail height and depth
FENCE_GATE = (-4.0, 4.0)                    # x range of the south gap

# Parking stalls: 9 ft (2.743 m) wide, 5.5 m deep, 0.1 m painted lines.
STALL_WIDTH = 2.743
STALL_DEPTH = 5.5
LINE_WIDTH = 0.1

MAX_GRAY = 110
ELEVATED_MIN_Z = 2.0

# ---- colours (RGB diffuse; ambient is set equal) ----------------------------

COLOURS = {
    'asphalt':   (0.27, 0.27, 0.28),
    'stall':     (0.50, 0.42, 0.08),        # muted yellow, gray ~103
    'grass':     (0.24, 0.36, 0.16),
    'concrete':  (0.42, 0.42, 0.40),
    'pole':      (0.18, 0.18, 0.19),
    'fence':     (0.06, 0.06, 0.06),
    'trunk':     (0.28, 0.20, 0.13),
    'leaves':    (0.13, 0.30, 0.10),
    'leaves_lt': (0.20, 0.38, 0.14),
    'tent_leg':  (0.35, 0.35, 0.36),
    'tent_roof': (0.88, 0.88, 0.86),        # ELEVATED only
    'table':     (0.30, 0.30, 0.32),
    'car_red':   (0.45, 0.08, 0.08),
    'car_blue':  (0.10, 0.18, 0.40),
    'car_grey':  (0.38, 0.38, 0.40),
    'car_black': (0.08, 0.08, 0.09),
    'car_green': (0.12, 0.28, 0.18),
    'glass':     (0.10, 0.12, 0.14),
}

ELEVATED = {'tent_roof'}

# ---- stall rows ----------------------------------------------------------------
# (name, x0, y0, count, along, depth_dir)
#   along:     'x' -> stalls side by side along +x; 'y' -> along +y
#   depth_dir: +1 / -1, which way the stall extends from the (x0, y0) line
# The stall row's back line runs along `along`; separators run depth_dir.

STALL_ROWS = [
    ('south',   -30.0, -15.0, 22, 'x', +1),
    ('north',   -30.0,  49.0, 22, 'x', -1),
    ('east',     35.0,  -2.0, 13, 'y', -1),
    ('west',    -35.0,   6.0,  5, 'y', +1),
]

# Fraction of stalls with a parked car (seeded, so the layout is fixed).
CAR_FILL = 0.35
CAR_COLOURS = ['car_red', 'car_blue', 'car_grey', 'car_black', 'car_green']

# ---- light poles: (name, x, y) -------------------------------------------------

POLES = [
    ('pole_sw', -26.0, -7.0),
    ('pole_s',    0.0, -7.5),
    ('pole_se',  26.0, -7.0),
    ('pole_w',  -27.5, 16.3),
    ('pole_e',   27.5, 16.3),
    ('pole_nw', -26.0, 40.0),
    ('pole_n',    0.0, 40.5),
    ('pole_ne',  26.0, 40.0),
]
POLE_BASE_RADIUS = 0.35
POLE_BASE_HEIGHT = 0.8
POLE_RADIUS = 0.1
POLE_HEIGHT = 9.0

# ---- tents: (name, x, y, yaw) - 3 x 3 m pop-up canopies with a table ----------
# Team pit area in the west of the lot, and two on the grass north of the fence.

TENTS = [
    ('tent_pit_1', -31.5, 23.0, 0.0),
    ('tent_pit_2', -31.5, 27.0, 0.0),
    ('tent_pit_3', -31.5, 31.0, 0.0),
    ('tent_pit_4', -31.5, 35.0, 0.0),
    ('tent_grass_1', -8.0, 55.0, 0.2),
    ('tent_grass_2',  6.0, 56.0, -0.1),
]
TENT_SIZE = 3.0
TENT_LEG_HEIGHT = 2.2
TENT_LEG_RADIUS = 0.025
TENT_VALANCE = 0.25

# ---- trees -------------------------------------------------------------------
# Rows of trees in the grass outside the fence, jittered (seeded). Plus a few
# bushes right behind the fence, low enough for the flat lidar ring to hit.

TREE_SEED = 7
TREE_BAND = 5.0                             # first tree row this far past the fence
TREE_SPACING = 7.0
TREE_ROWS = 2
TREE_ROW_GAP = 7.0
BUSH_COUNT = 24

# ---- generated layout ----------------------------------------------------------


def stall_lines():
    """(name, cx, cy, sx, sy) painted rectangles for every stall row."""
    lines = []
    for name, x0, y0, count, along, depth_dir in STALL_ROWS:
        length = count * STALL_WIDTH
        if along == 'x':
            # back line, then separators running in y
            lines.append((f'stall_{name}_back', x0 + length / 2, y0,
                          length, LINE_WIDTH))
            for i in range(count + 1):
                lines.append((f'stall_{name}_{i}', x0 + i * STALL_WIDTH,
                              y0 + depth_dir * STALL_DEPTH / 2,
                              LINE_WIDTH, STALL_DEPTH))
        else:
            lines.append((f'stall_{name}_back', x0, y0 + length / 2,
                          LINE_WIDTH, length))
            for i in range(count + 1):
                lines.append((f'stall_{name}_{i}',
                              x0 + depth_dir * STALL_DEPTH / 2,
                              y0 + i * STALL_WIDTH,
                              STALL_DEPTH, LINE_WIDTH))
    return lines


def parked_cars():
    """(name, x, y, yaw, colour) for the stalls that get a car."""
    rng = random.Random(TREE_SEED)
    cars = []
    for name, x0, y0, count, along, depth_dir in STALL_ROWS:
        for i in range(count):
            if rng.random() >= CAR_FILL:
                continue
            centre = (i + 0.5) * STALL_WIDTH
            # nose 0.5 m from the back line
            inset = 0.5 + 2.25
            if along == 'x':
                x, y = x0 + centre, y0 + depth_dir * inset
                yaw = math.pi / 2
            else:
                x, y = x0 + depth_dir * inset, y0 + centre
                yaw = 0.0
            cars.append((f'car_{name}_{i}', x, y, yaw, rng.choice(CAR_COLOURS)))
    return cars


CAR_LENGTH, CAR_WIDTH = 4.5, 1.8


def trees():
    """(name, x, y, trunk_r, trunk_h, canopy_r, colour) outside the fence."""
    rng = random.Random(TREE_SEED + 1)
    out = []
    x0, x1, y0, y1 = LOT
    for row in range(TREE_ROWS):
        off = TREE_BAND + row * TREE_ROW_GAP
        rx0, rx1, ry0, ry1 = x0 - off, x1 + off, y0 - off, y1 + off
        # walk the rectangle's perimeter
        perimeter = 2 * ((rx1 - rx0) + (ry1 - ry0))
        n = int(perimeter / TREE_SPACING)
        for k in range(n):
            s = (k + 0.5 * row) * perimeter / n
            if s < rx1 - rx0:
                x, y = rx0 + s, ry0
            elif s < (rx1 - rx0) + (ry1 - ry0):
                x, y = rx1, ry0 + s - (rx1 - rx0)
            elif s < 2 * (rx1 - rx0) + (ry1 - ry0):
                x, y = rx1 - (s - (rx1 - rx0) - (ry1 - ry0)), ry1
            else:
                x, y = rx0, ry1 - (s - 2 * (rx1 - rx0) - (ry1 - ry0))
            x += rng.uniform(-1.5, 1.5)
            y += rng.uniform(-1.5, 1.5)
            if not (WORLD[0] + 3 < x < WORLD[1] - 3 and WORLD[2] + 3 < y < WORLD[3] - 3):
                continue
            out.append((f'tree_{row}_{k}', x, y,
                        rng.uniform(0.15, 0.3), rng.uniform(2.5, 4.5),
                        rng.uniform(2.0, 3.5),
                        rng.choice(['leaves', 'leaves_lt'])))
    return out


def bushes():
    """(name, x, y, radius) just outside the fence."""
    rng = random.Random(TREE_SEED + 2)
    x0, x1, y0, y1 = LOT
    out = []
    for k in range(BUSH_COUNT):
        side = k % 4
        r = rng.uniform(0.9, 1.4)
        off = 1.0 + r
        if side == 0:
            x, y = rng.uniform(x0 + 3, x1 - 3), y1 + off
        elif side == 1:
            x, y = x1 + off, rng.uniform(y0 + 3, y1 - 3)
        elif side == 2:
            x, y = rng.uniform(x0 + 3, x1 - 3), y0 - off
            if FENCE_GATE[0] - 3 < x < FENCE_GATE[1] + 3:
                continue
        else:
            x, y = x0 - off, rng.uniform(y0 + 3, y1 - 3)
        # Skip spots taken by another bush or a tent on the grass.
        if any(math.hypot(x - bx, y - by) < r + br + OBJECT_GAP
               for _n, bx, by, br in out):
            continue
        if any(math.hypot(x - tx, y - ty) < r + TENT_SIZE + OBJECT_GAP
               for _n, tx, ty, _yaw in TENTS):
            continue
        out.append((f'bush_{k}', x, y, r))
    return out


# ---- validation --------------------------------------------------------------


def footprints():
    """(name, zone, polygon). zone: 'lot' inside the fence, 'grass' outside."""
    out = []
    for name, x, y in POLES:
        out.append((name, 'lot', circle_poly(x, y, POLE_BASE_RADIUS)))
    for name, x, y, yaw in TENTS:
        zone = 'lot' if point_rect_dist(x, y, LOT) == 0 else 'grass'
        out.append((name, zone, box_poly(x, y, TENT_SIZE, TENT_SIZE, yaw)))
    for name, x, y, yaw, _c in parked_cars():
        out.append((name, 'lot', box_poly(x, y, CAR_LENGTH, CAR_WIDTH, yaw)))
    for name, x, y, trunk_r, _h, _cr, _c in trees():
        out.append((name, 'grass', circle_poly(x, y, trunk_r)))
    for name, x, y, r in bushes():
        out.append((name, 'grass', circle_poly(x, y, r)))
    return out


def validate():
    errors = []

    for key, rgb in COLOURS.items():
        if key not in ELEVATED and gray(rgb) > MAX_GRAY:
            errors.append(f'colour {key!r} grayscale {gray(rgb):.0f} > {MAX_GRAY}')
    if TENT_LEG_HEIGHT < ELEVATED_MIN_Z:
        errors.append(f'tent roofs (ELEVATED colour) sit below {ELEVATED_MIN_Z} m')

    prints = footprints()
    names = [p[0] for p in prints]
    dupes = {n for n in names if names.count(n) > 1}
    if dupes:
        errors.append(f'duplicate names: {sorted(dupes)}')

    keepout = rect_poly(TRACK[0] - KEEPOUT_MARGIN, TRACK[1] + KEEPOUT_MARGIN,
                        TRACK[2] - KEEPOUT_MARGIN, TRACK[3] + KEEPOUT_MARGIN)
    fence_in = (LOT[0] + 0.3, LOT[1] - 0.3, LOT[2] + 0.3, LOT[3] - 0.3)
    fence_out = rect_poly(LOT[0] - 0.3, LOT[1] + 0.3, LOT[2] - 0.3, LOT[3] + 0.3)

    for name, zone, poly in prints:
        if polys_overlap(poly, keepout):
            errors.append(f'{name} is inside the track keep-out')
        if zone == 'lot':
            if any(point_rect_dist(px, py, fence_in) > 1e-9 for px, py in poly):
                errors.append(f'{name} crosses the fence')
        else:
            if polys_overlap(poly, fence_out):
                errors.append(f'{name} is inside (or on) the fence')
            if any(point_rect_dist(px, py, WORLD) > 1e-9 for px, py in poly):
                errors.append(f'{name} is outside the world')

    for i in range(len(prints)):
        for j in range(i + 1, len(prints)):
            a, b = prints[i], prints[j]
            # Bushes and tree trunks overlapping canopies is fine; only
            # check solid footprints, with a gap.
            if polys_overlap(_grow(a[2], OBJECT_GAP / 2), _grow(b[2], OBJECT_GAP / 2)):
                errors.append(f'{a[0]} overlaps {b[0]}')

    for name, cx, cy, sx, sy in stall_lines():
        poly = rect_poly(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2)
        if polys_overlap(poly, rect_poly(*TRACK)):
            errors.append(f'{name} overlaps the track mesh')
        if any(point_rect_dist(px, py, LOT) > 1e-9 for px, py in poly):
            errors.append(f'{name} is outside the lot')

    return errors


def _grow(poly, d):
    """Push a convex polygon's vertices away from its centroid by d."""
    cx = sum(p[0] for p in poly) / len(poly)
    cy = sum(p[1] for p in poly) / len(poly)
    out = []
    for px, py in poly:
        vx, vy = px - cx, py - cy
        n = math.hypot(vx, vy) or 1.0
        out.append((px + vx / n * d, py + vy / n * d))
    return out


# ---- fence mesh ------------------------------------------------------------------


def fence_segments():
    """Straight fence runs as ((x0, y0), (x1, y1)), with the south gate gap."""
    x0, x1, y0, y1 = LOT
    return [
        ((x0, y0), (FENCE_GATE[0], y0)),
        ((FENCE_GATE[1], y0), (x1, y0)),
        ((x1, y0), (x1, y1)),
        ((x1, y1), (x0, y1)),
        ((x0, y1), (x0, y0)),
    ]


class Obj:
    """Minimal OBJ writer for axis-aligned-in-segment boxes."""

    def __init__(self):
        self.v = []
        self.vn = []
        self.f = []

    def box(self, cx, cy, cz, sx, sy, sz, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        base = len(self.v) + 1
        for dz in (-sz / 2, sz / 2):
            for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2),
                           (sx / 2, sy / 2), (-sx / 2, sy / 2)):
                self.v.append((cx + dx * c - dy * s, cy + dx * s + dy * c, cz + dz))
        # Without normals the mesh is lit wrongly and renders white,
        # whatever its material, so every face carries its own.
        b = base
        normals = [(0, 0, -1), (0, 0, 1), (s, -c, 0),
                   (c, s, 0), (-s, c, 0), (-c, -s, 0)]
        quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                 (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
        for q, n in zip(quads, normals):
            self.vn.append(n)
            self.f.append((tuple(b + i for i in q), len(self.vn)))

    def text(self, mtl_file, mtl_name):
        # Gazebo ignores the SDF <material> on an OBJ without its own
        # material and renders it white, so the colour must live in the .mtl.
        lines = ['# GENERATED by tools/generate_carpark_world.py',
                 f'mtllib {mtl_file}', f'usemtl {mtl_name}']
        lines += [f'v {x:.4f} {y:.4f} {z:.4f}' for x, y, z in self.v]
        lines += [f'vn {x:.4f} {y:.4f} {z:.4f}' for x, y, z in self.vn]
        lines += ['f ' + ' '.join(f'{i}//{n}' for i in q) for q, n in self.f]
        return '\n'.join(lines) + '\n'


def build_fence():
    obj = Obj()
    pickets = 0
    for (ax, ay), (bx, by) in fence_segments():
        length = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax)
        ux, uy = (bx - ax) / length, (by - ay) / length

        def at(d):
            return ax + ux * d, ay + uy * d

        # rails: bottom, top
        mx, my = at(length / 2)
        for z in (0.12, FENCE_HEIGHT - FENCE_RAIL / 2):
            obj.box(mx, my, z, length, FENCE_RAIL, FENCE_RAIL, yaw)
        # posts
        n_posts = max(1, round(length / FENCE_POST_SPACING))
        for i in range(n_posts + 1):
            px, py = at(i * length / n_posts)
            obj.box(px, py, FENCE_HEIGHT / 2 + 0.05, FENCE_POST, FENCE_POST,
                    FENCE_HEIGHT + 0.1, yaw)
        # pickets
        n = int(length / FENCE_SPACING)
        for i in range(1, n):
            px, py = at(i * FENCE_SPACING)
            obj.box(px, py, FENCE_HEIGHT / 2, FENCE_PICKET, FENCE_PICKET,
                    FENCE_HEIGHT, yaw)
            pickets += 1
    return obj, pickets


# ---- SDF emission ------------------------------------------------------------


def material(colour, indent='        '):
    r, g, b = COLOURS[colour]
    rgba = f'{r:.2f} {g:.2f} {b:.2f} 1'
    return (f'{indent}<material>\n'
            f'{indent}  <ambient>{rgba}</ambient>\n'
            f'{indent}  <diffuse>{rgba}</diffuse>\n'
            f'{indent}  <specular>0.05 0.05 0.05 1</specular>\n'
            f'{indent}</material>\n')


def visual(name, pose, geometry, colour, collide=True, shadows=True):
    out = (f'      <visual name="{name}">\n'
           f'        <pose>{pose}</pose>\n'
           + ('' if shadows else '        <cast_shadows>false</cast_shadows>\n') +
           f'        <geometry>{geometry}</geometry>\n'
           f'{material(colour)}'
           f'      </visual>\n')
    if collide:
        out += (f'      <collision name="{name}_collision">\n'
                f'        <pose>{pose}</pose>\n'
                f'        <geometry>{geometry}</geometry>\n'
                f'      </collision>\n')
    return out


def link(name, pose, body):
    return (f'    <link name="{name}">\n'
            f'      <pose>{pose}</pose>\n'
            f'{body}'
            f'    </link>\n')


def box(sx, sy, sz):
    return f'<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>'


def cyl(r, h):
    return f'<cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder>'


def plane(sx, sy):
    return f'<plane><normal>0 0 1</normal><size>{sx:.3f} {sy:.3f}</size></plane>'


def build_sdf():
    wx0, wx1, wy0, wy1 = WORLD
    lx0, lx1, ly0, ly1 = LOT
    parts = []

    # Base ground: one collision plane, same friction as orange_igvc's
    # ground. Visuals are sunk/raised by millimetres so the track texture
    # (z = 0) wins and nothing z-fights.
    parts.append('    <!--ground-->\n')
    parts.append(
        f'    <link name="ground">\n'
        f'      <pose>{(wx0 + wx1) / 2:.3f} {(wy0 + wy1) / 2:.3f} 0 0 0 0</pose>\n'
        f'      <visual name="grass">\n'
        f'        <pose>0 0 -0.02 0 0 0</pose>\n'
        f'        <cast_shadows>false</cast_shadows>\n'
        f'        <geometry>{plane(wx1 - wx0, wy1 - wy0)}</geometry>\n'
        f'{material("grass")}'
        f'      </visual>\n'
        f'      <collision name="collision">\n'
        f'        <geometry>{plane(wx1 - wx0, wy1 - wy0)}</geometry>\n'
        f'        <surface>\n'
        f'          <friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>\n'
        f'          <bounce/>\n'
        f'          <contact><ode/></contact>\n'
        f'        </surface>\n'
        f'      </collision>\n'
        f'    </link>\n')

    # Asphalt lot under the track (track mesh at z = 0 covers its part).
    parts.append(link(
        'lot', f'{(lx0 + lx1) / 2:.3f} {(ly0 + ly1) / 2:.3f} -0.01 0 0 0',
        visual('asphalt', '0 0 0 0 0 0', plane(lx1 - lx0, ly1 - ly0),
               'asphalt', collide=False, shadows=False)))

    parts.append('    <!--stall lines (visual only)-->\n')
    body = ''.join(
        visual(name, f'{cx:.3f} {cy:.3f} 0.003 0 0 0', plane(sx, sy),
               'stall', collide=False, shadows=False)
        for name, cx, cy, sx, sy in stall_lines())
    parts.append(link('stall_lines', '0 0 0 0 0 0', body))

    parts.append('    <!--fence: pickets as one mesh, simple collision walls-->\n')
    body = visual('fence', '0 0 0 0 0 0',
                  '<mesh><uri>model://esda_carpark/meshes/fence.obj</uri></mesh>',
                  'fence', collide=False)
    for i, ((ax, ay), (bx, by)) in enumerate(fence_segments()):
        length = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax)
        body += (f'      <collision name="fence_wall_{i}">\n'
                 f'        <pose>{(ax + bx) / 2:.3f} {(ay + by) / 2:.3f} '
                 f'{FENCE_HEIGHT / 2:.3f} 0 0 {yaw:.4f}</pose>\n'
                 f'        <geometry>{box(length, 0.05, FENCE_HEIGHT)}</geometry>\n'
                 f'      </collision>\n')
    parts.append(link('fence', '0 0 0 0 0 0', body))

    parts.append('    <!--light poles-->\n')
    for name, x, y in POLES:
        body = (visual('base', f'0 0 {POLE_BASE_HEIGHT / 2} 0 0 0',
                       cyl(POLE_BASE_RADIUS, POLE_BASE_HEIGHT), 'concrete')
                + visual('pole', f'0 0 {POLE_BASE_HEIGHT + POLE_HEIGHT / 2} 0 0 0',
                         cyl(POLE_RADIUS, POLE_HEIGHT), 'pole')
                + visual('lamp', f'0.4 0 {POLE_BASE_HEIGHT + POLE_HEIGHT} 0 0 0',
                         box(0.9, 0.3, 0.15), 'pole', collide=False))
        parts.append(link(name, f'{x} {y} 0 0 0 0', body))

    parts.append('    <!--parked cars-->\n')
    for name, x, y, yaw, colour in parked_cars():
        body = (visual('body', '0 0 0.55 0 0 0', box(CAR_LENGTH, CAR_WIDTH, 0.8), colour)
                + visual('cabin', '-0.2 0 1.2 0 0 0', box(2.4, 1.6, 0.5), 'glass'))
        for wx in (-1.4, 1.4):
            for wy in (-0.8, 0.8):
                body += visual(f'wheel_{wx}_{wy}', f'{wx} {wy} 0.33 1.5708 0 0',
                               cyl(0.33, 0.22), 'car_black', collide=False)
        parts.append(link(name, f'{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}', body))

    parts.append('    <!--tents-->\n')
    h = TENT_SIZE / 2 - 0.05
    for name, x, y, yaw in TENTS:
        body = ''
        for i, (lx, ly) in enumerate(((-h, -h), (h, -h), (h, h), (-h, h))):
            body += visual(f'leg_{i}', f'{lx} {ly} {TENT_LEG_HEIGHT / 2} 0 0 0',
                           cyl(TENT_LEG_RADIUS, TENT_LEG_HEIGHT), 'tent_leg')
        roof_z = TENT_LEG_HEIGHT + 0.25
        body += visual('roof_low', f'0 0 {TENT_LEG_HEIGHT + 0.02} 0 0 0',
                       box(TENT_SIZE, TENT_SIZE, 0.04), 'tent_roof', collide=False)
        body += visual('roof_peak', f'0 0 {roof_z} 0 0 0',
                       box(TENT_SIZE * 0.5, TENT_SIZE * 0.5, 0.5), 'tent_roof',
                       collide=False)
        for i, (vx, vy, sx, sy) in enumerate(((0, -h, TENT_SIZE, 0.02),
                                              (0, h, TENT_SIZE, 0.02),
                                              (-h, 0, 0.02, TENT_SIZE),
                                              (h, 0, 0.02, TENT_SIZE))):
            body += visual(f'valance_{i}',
                           f'{vx} {vy} {TENT_LEG_HEIGHT - TENT_VALANCE / 2} 0 0 0',
                           box(sx, sy, TENT_VALANCE), 'tent_roof', collide=False)
        body += visual('table', '0 -0.6 0.37 0 0 0', box(1.8, 0.75, 0.74), 'table')
        parts.append(link(name, f'{x} {y} 0 0 0 {yaw}', body))

    parts.append('    <!--trees-->\n')
    for name, x, y, tr, th, cr, colour in trees():
        body = (visual('trunk', f'0 0 {th / 2:.3f} 0 0 0', cyl(tr, th), 'trunk')
                + visual('canopy', f'0 0 {th + cr * 0.7:.3f} 0 0 0',
                         f'<sphere><radius>{cr:.3f}</radius></sphere>', colour,
                         collide=False))
        parts.append(link(name, f'{x:.3f} {y:.3f} 0 0 0 0', body))

    parts.append('    <!--bushes-->\n')
    for name, x, y, r in bushes():
        parts.append(link(name, f'{x:.3f} {y:.3f} 0 0 0 0',
                          visual('bush', f'0 0 {r * 0.7:.3f} 0 0 0',
                                 f'<sphere><radius>{r:.3f}</radius></sphere>',
                                 'leaves_lt')))

    return ('<?xml version="1.0"?>\n'
            '<!-- GENERATED by tools/generate_carpark_world.py - edit the layout there, not here. -->\n'
            '<sdf version="1.6">\n'
            '  <model name="esda_carpark">\n'
            '    <static>true</static>\n'
            + ''.join(parts) +
            '  </model>\n'
            '</sdf>\n')


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>esda_carpark</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>IGVC-style parking lot (asphalt, yellow stalls, light poles, parked cars, tents, picket fence, trees) for the orange_igvc track. Generated by tools/generate_carpark_world.py.</description>
</model>
"""


def main():
    errors = validate()
    if errors:
        print('Layout check FAILED:')
        for e in errors:
            print(f'  - {e}')
        return 1

    os.makedirs(os.path.join(MODEL_DIR, 'meshes'), exist_ok=True)
    fence, pickets = build_fence()
    with open(os.path.join(MODEL_DIR, 'meshes', 'fence.obj'), 'w', newline='\n') as f:
        f.write(fence.text('fence.mtl', 'fence'))
    r, g, b = COLOURS['fence']
    with open(os.path.join(MODEL_DIR, 'meshes', 'fence.mtl'), 'w', newline='\n') as f:
        f.write('# GENERATED by tools/generate_carpark_world.py\n'
                'newmtl fence\n'
                f'Ka {r:.2f} {g:.2f} {b:.2f}\n'
                f'Kd {r:.2f} {g:.2f} {b:.2f}\n'
                'Ks 0.05 0.05 0.05\n'
                'd 1.0\n')
    with open(os.path.join(MODEL_DIR, 'model.sdf'), 'w', newline='\n') as f:
        f.write(build_sdf())
    with open(os.path.join(MODEL_DIR, 'model.config'), 'w', newline='\n') as f:
        f.write(MODEL_CONFIG)

    print(f'Wrote {os.path.relpath(MODEL_DIR, REPO_ROOT)}/')
    print(f'  {len(stall_lines())} stall lines, {len(parked_cars())} cars, '
          f'{len(POLES)} poles, {len(TENTS)} tents, {len(trees())} trees, '
          f'{len(bushes())} bushes, {pickets} fence pickets')
    return 0


if __name__ == '__main__':
    sys.exit(main())
