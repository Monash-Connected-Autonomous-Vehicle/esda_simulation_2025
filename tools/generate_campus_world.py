#!/usr/bin/env python3
"""
Generate the `esda_campus` Gazebo model: the ~100 x 100 m surroundings that
worlds/igvc_campus.sdf places around the unchanged orange_igvc track.

Stdlib only - no ROS or Gazebo needed.

    python3 tools/generate_campus_world.py

Writes src/esda_simulation_2025/worlds/models/esda_campus/model.sdf. Edit the
layout tables below and re-run; the output is committed alongside this script.

All coordinates are WORLD frame. orange_igvc is included at (0, 16.3), so the
track occupies x in [-21.5, 21.5], y in [-2.2, 34.8] and the robot still spawns
at (11, 0).

The generator refuses to write if the layout breaks any of:
  * buildings must not overlap each other (except parts of one cluster),
    the ring/radial roads, car parks, or the track keep-out zone;
  * buildings must sit inside the perimeter wall;
  * every colour must stay at or below MAX_GRAY. lane_detection.py thresholds
    grayscale at 130 and the track texture averages ~128, so anything brighter
    would be picked up as a lane line.
"""

import math
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH = os.path.join(REPO_ROOT, 'src', 'esda_simulation_2025', 'worlds',
                        'models', 'esda_campus', 'model.sdf')

# ---- geometry --------------------------------------------------------------

TRACK = (-21.5, 21.5, -2.2, 34.8)           # (xmin, xmax, ymin, ymax)
APRON_WIDTH = 4.0
KEEPOUT_MARGIN = 6.0                        # building clearance beyond the apron
WORLD = (-50.0, 50.0, -33.7, 66.3)          # 100 x 100 m, centred on the track
WALL_THICKNESS = 0.3
WALL_HEIGHT = 2.0
BUILDING_GAP = 1.0                          # min gap between unrelated footprints
WALL_MARGIN = 0.5                           # min gap between buildings and the wall

MAX_GRAY = 110

# ---- colours (RGB diffuse; ambient is set equal) ----------------------------

COLOURS = {
    'base':     (0.25, 0.25, 0.26),
    'apron':    (0.38, 0.38, 0.37),
    'road':     (0.20, 0.20, 0.21),
    'carpark':  (0.29, 0.29, 0.30),
    'brick':    (0.42, 0.20, 0.16),
    'slate':    (0.30, 0.33, 0.38),
    'tan':      (0.45, 0.38, 0.28),
    'olive':    (0.32, 0.34, 0.24),
    'charcoal': (0.22, 0.22, 0.24),
    'teal':     (0.22, 0.34, 0.36),
    'silo':     (0.40, 0.40, 0.42),
    'wall':     (0.30, 0.28, 0.26),
}

# ---- pavement (visual only; collision is the single base plane) ------------
# (name, xmin, xmax, ymin, ymax, z, colour)

ax0, ax1, ay0, ay1 = (TRACK[0] - APRON_WIDTH, TRACK[1] + APRON_WIDTH,
                      TRACK[2] - APRON_WIDTH, TRACK[3] + APRON_WIDTH)
RING = 8.0
rx0, rx1, ry0, ry1 = ax0 - RING, ax1 + RING, ay0 - RING, ay1 + RING

PAVEMENT = [
    # 4 m apron hugging the track mesh edge (never overlaps the track itself)
    ('apron_south', ax0, ax1, ay0, TRACK[2], 0.002, 'apron'),
    ('apron_north', ax0, ax1, TRACK[3], ay1, 0.002, 'apron'),
    ('apron_west', ax0, TRACK[0], TRACK[2], TRACK[3], 0.002, 'apron'),
    ('apron_east', TRACK[1], ax1, TRACK[2], TRACK[3], 0.002, 'apron'),
    # 8 m ring road around the apron
    ('ring_south', rx0, rx1, ry0, ay0, 0.003, 'road'),
    ('ring_north', rx0, rx1, ay1, ry1, 0.003, 'road'),
    ('ring_west', rx0, ax0, ay0, ay1, 0.003, 'road'),
    ('ring_east', ax1, rx1, ay0, ay1, 0.003, 'road'),
    # 12 m radial roads out to the perimeter
    ('radial_south', -6.0, 6.0, WORLD[2], ry0, 0.003, 'road'),
    ('radial_north', -6.0, 6.0, ry1, WORLD[3], 0.003, 'road'),
    # car parks
    ('carpark_southeast', 8.0, 22.0, -31.0, -17.0, 0.003, 'carpark'),
    ('carpark_northeast', 8.0, 22.0, 49.0, 64.0, 0.003, 'carpark'),
]

# ---- buildings -------------------------------------------------------------
# Boxes:     (name, x, y, w, d, h, yaw, colour, cluster)
# Cylinders: (name, x, y, radius, h, colour)
# `cluster` lets parts of an L-shape / courtyard touch each other.

BOXES = [
    # east band
    ('east_office',       41.5, -26.0, 12.0, 10.0,  8.0,  0.00, 'brick',    None),
    ('east_lab',          42.0, -10.0, 10.0, 14.0, 12.0,  0.10, 'slate',    None),
    ('east_l_main',       40.0,   8.0,  8.0, 14.0,  6.0,  0.00, 'tan',      'east_l'),
    ('east_l_wing',       46.0,   4.0,  4.0,  6.0,  6.0,  0.00, 'tan',      'east_l'),
    ('east_shed',         38.0,  25.5,  4.0,  5.0,  3.0,  0.00, 'olive',    None),
    ('east_hall',         41.0,  35.0, 11.0,  9.0, 10.0, -0.15, 'teal',     None),
    ('east_tower',        42.0,  53.0, 13.0, 12.0, 15.0,  0.00, 'charcoal', None),
    ('east_kiosk',        38.0,  63.0,  6.0,  4.0,  3.0,  0.00, 'brick',    None),
    # west band
    ('west_depot',       -42.0, -24.0, 14.0, 14.0, 10.0,  0.00, 'slate',    None),
    ('west_court_main',  -40.0,  -5.0,  8.0, 16.0,  7.0,  0.00, 'brick',    'west_court'),
    ('west_court_wing',  -46.0,  -9.0,  4.0,  8.0,  7.0,  0.00, 'brick',    'west_court'),
    ('west_angled',      -42.0,  14.0, 12.0,  8.0,  5.0,  0.25, 'olive',    None),
    ('west_shed',        -37.5,  34.0,  4.0,  5.0,  3.0,  0.00, 'tan',      None),
    ('west_library',     -41.0,  44.0, 12.0, 10.0,  9.0,  0.00, 'teal',     None),
    ('west_block',       -40.0,  58.0, 14.0, 12.0, 13.0, -0.10, 'charcoal', None),
    # south band
    ('south_workshop',   -27.0, -24.0, 10.0, 12.0,  6.0,  0.00, 'tan',      None),
    ('south_annex',      -14.0, -25.0, 10.0, 10.0,  9.0,  0.20, 'slate',    None),
    ('south_garage',      28.0, -24.0,  9.0, 14.0,  7.0,  0.00, 'charcoal', None),
    # north band
    ('north_centre',     -24.0,  56.0, 12.0, 12.0, 11.0,  0.00, 'brick',    None),
    ('north_cafe',       -12.0,  55.0,  8.0, 10.0,  6.0,  0.00, 'olive',    None),
    ('north_store',       28.0,  56.0, 10.0, 14.0,  8.0,  0.00, 'teal',     None),
]

CYLINDERS = [
    ('east_water_tower',  44.0,  22.0, 3.0, 14.0, 'silo'),
    ('west_silo',        -43.0,  27.0, 2.5, 10.0, 'silo'),
    ('west_silo_small',  -46.5,  32.0, 1.8,  8.0, 'silo'),
]

# ---- 2D helpers --------------------------------------------------------------


def rect_poly(xmin, xmax, ymin, ymax):
    return [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]


def box_poly(x, y, w, d, yaw, grow=0.0):
    hw, hd = w / 2 + grow, d / 2 + grow
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + px * c - py * s, y + px * s + py * c)
            for px, py in ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd))]


def circle_poly(x, y, r, grow=0.0, n=24):
    # Circumscribed polygon, so it fully contains the circle.
    R = (r + grow) / math.cos(math.pi / n)
    return [(x + R * math.cos(2 * math.pi * i / n),
             y + R * math.sin(2 * math.pi * i / n)) for i in range(n)]


def polys_overlap(a, b, eps=1e-9):
    """Separating-axis test for convex polygons; touching is not overlap."""
    for poly in (a, b):
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            nx, ny = y2 - y1, x1 - x2
            pa = [nx * px + ny * py for px, py in a]
            pb = [nx * px + ny * py for px, py in b]
            if max(pa) <= min(pb) + eps or max(pb) <= min(pa) + eps:
                return False
    return True


def point_rect_dist(px, py, rect):
    xmin, xmax, ymin, ymax = rect
    dx = max(xmin - px, 0.0, px - xmax)
    dy = max(ymin - py, 0.0, py - ymax)
    return math.hypot(dx, dy)


def gray(rgb):
    r, g, b = rgb
    return (0.299 * r + 0.587 * g + 0.114 * b) * 255


# ---- validation --------------------------------------------------------------


def footprints(grow):
    """(name, cluster, polygon) for every building, grown by `grow` metres."""
    out = []
    for name, x, y, w, d, _h, yaw, _c, cluster in BOXES:
        out.append((name, cluster, box_poly(x, y, w, d, yaw, grow)))
    for name, x, y, r, _h, _c in CYLINDERS:
        out.append((name, None, circle_poly(x, y, r, grow)))
    return out


def validate():
    errors = []

    for key, rgb in COLOURS.items():
        if gray(rgb) > MAX_GRAY:
            errors.append(f'colour {key!r} grayscale {gray(rgb):.0f} > {MAX_GRAY}')

    names = [b[0] for b in BOXES] + [c[0] for c in CYLINDERS] + [p[0] for p in PAVEMENT]
    dupes = {n for n in names if names.count(n) > 1}
    if dupes:
        errors.append(f'duplicate names: {sorted(dupes)}')

    # Buildings vs each other, with BUILDING_GAP between unrelated footprints.
    half = footprints(BUILDING_GAP / 2)
    for i in range(len(half)):
        for j in range(i + 1, len(half)):
            ni, ci, pi = half[i]
            nj, cj, pj = half[j]
            if ci is not None and ci == cj:
                continue
            if polys_overlap(pi, pj):
                errors.append(f'{ni} is within {BUILDING_GAP} m of {nj}')

    # Buildings vs track keep-out, the whole ring-road block, and all pavement.
    keepout = (ax0 - KEEPOUT_MARGIN, ax1 + KEEPOUT_MARGIN,
               ay0 - KEEPOUT_MARGIN, ay1 + KEEPOUT_MARGIN)
    zones = [('track keep-out', rect_poly(*keepout)),
             ('ring road block', rect_poly(rx0, rx1, ry0, ry1))]
    zones += [(p[0], rect_poly(*p[1:5])) for p in PAVEMENT]
    for name, _cluster, poly in footprints(BUILDING_GAP):
        for zname, zpoly in zones:
            if polys_overlap(poly, zpoly):
                errors.append(f'{name} is within {BUILDING_GAP} m of {zname}')

    # Buildings inside the wall.
    inner = (WORLD[0] + WALL_THICKNESS + WALL_MARGIN, WORLD[1] - WALL_THICKNESS - WALL_MARGIN,
             WORLD[2] + WALL_THICKNESS + WALL_MARGIN, WORLD[3] - WALL_THICKNESS - WALL_MARGIN)
    for name, _cluster, poly in footprints(0.0):
        if any(point_rect_dist(px, py, inner) > 1e-9 for px, py in poly):
            errors.append(f'{name} is outside the perimeter (or within {WALL_MARGIN} m of the wall)')

    # Pavement at the same height must not overlap (z-fighting), and stays in bounds.
    for i in range(len(PAVEMENT)):
        a = PAVEMENT[i]
        if a[1] < WORLD[0] or a[2] > WORLD[1] or a[3] < WORLD[2] or a[4] > WORLD[3]:
            errors.append(f'{a[0]} extends past the world edge')
        if polys_overlap(rect_poly(*a[1:5]), rect_poly(*TRACK)):
            errors.append(f'{a[0]} overlaps the track mesh')
        for b in PAVEMENT[i + 1:]:
            if a[5] == b[5] and polys_overlap(rect_poly(*a[1:5]), rect_poly(*b[1:5])):
                errors.append(f'{a[0]} overlaps {b[0]} at the same height')

    return errors


# ---- SDF emission ------------------------------------------------------------


def material(colour):
    r, g, b = COLOURS[colour]
    rgba = f'{r:.2f} {g:.2f} {b:.2f} 1'
    return (f'        <material>\n'
            f'          <ambient>{rgba}</ambient>\n'
            f'          <diffuse>{rgba}</diffuse>\n'
            f'          <specular>0.05 0.05 0.05 1</specular>\n'
            f'        </material>\n')


def solid_link(name, pose, geometry, colour):
    return (f'    <link name="{name}">\n'
            f'      <pose>{pose}</pose>\n'
            f'      <visual name="visual">\n'
            f'        <geometry>{geometry}</geometry>\n'
            f'{material(colour)}'
            f'      </visual>\n'
            f'      <collision name="collision">\n'
            f'        <geometry>{geometry}</geometry>\n'
            f'      </collision>\n'
            f'    </link>\n')


def plane_visual_link(name, cx, cy, z, sx, sy, colour):
    return (f'    <link name="{name}">\n'
            f'      <pose>{cx:.3f} {cy:.3f} {z} 0 0 0</pose>\n'
            f'      <visual name="visual">\n'
            f'        <cast_shadows>false</cast_shadows>\n'
            f'        <geometry><plane><normal>0 0 1</normal>'
            f'<size>{sx:.3f} {sy:.3f}</size></plane></geometry>\n'
            f'{material(colour)}'
            f'      </visual>\n'
            f'    </link>\n')


def build_sdf():
    wx0, wx1, wy0, wy1 = WORLD
    cx, cy = (wx0 + wx1) / 2, (wy0 + wy1) / 2
    sx, sy = wx1 - wx0, wy1 - wy0
    parts = []

    # Base ground: one collision plane for the whole area (same friction as
    # orange_igvc's ground), visual sunk 1 cm so the track texture wins.
    parts.append(
        f'    <!--ground-->\n'
        f'    <link name="ground">\n'
        f'      <pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>\n'
        f'      <visual name="visual">\n'
        f'        <pose>0 0 -0.01 0 0 0</pose>\n'
        f'        <cast_shadows>false</cast_shadows>\n'
        f'        <geometry><plane><normal>0 0 1</normal><size>{sx:.3f} {sy:.3f}</size></plane></geometry>\n'
        f'{material("base")}'
        f'      </visual>\n'
        f'      <collision name="collision">\n'
        f'        <geometry><plane><normal>0 0 1</normal><size>{sx:.3f} {sy:.3f}</size></plane></geometry>\n'
        f'        <surface>\n'
        f'          <friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>\n'
        f'          <bounce/>\n'
        f'          <contact><ode/></contact>\n'
        f'        </surface>\n'
        f'      </collision>\n'
        f'    </link>\n')

    parts.append('    <!--pavement (visual only)-->\n')
    for name, x0, x1, y0, y1, z, colour in PAVEMENT:
        parts.append(plane_visual_link(name, (x0 + x1) / 2, (y0 + y1) / 2, z,
                                       x1 - x0, y1 - y0, colour))

    parts.append('    <!--buildings-->\n')
    for name, x, y, w, d, h, yaw, colour, _cluster in BOXES:
        parts.append(solid_link(name, f'{x} {y} {h / 2} 0 0 {yaw}',
                                f'<box><size>{w} {d} {h}</size></box>', colour))
    for name, x, y, r, h, colour in CYLINDERS:
        parts.append(solid_link(name, f'{x} {y} {h / 2} 0 0 0',
                                f'<cylinder><radius>{r}</radius><length>{h}</length></cylinder>',
                                colour))

    parts.append('    <!--perimeter wall-->\n')
    t, hz = WALL_THICKNESS, WALL_HEIGHT / 2
    walls = [
        ('wall_south', cx, wy0 + t / 2, sx, t),
        ('wall_north', cx, wy1 - t / 2, sx, t),
        ('wall_west', wx0 + t / 2, cy, t, sy - 2 * t),
        ('wall_east', wx1 - t / 2, cy, t, sy - 2 * t),
    ]
    for name, x, y, w, d in walls:
        parts.append(solid_link(name, f'{x:.3f} {y:.3f} {hz} 0 0 0',
                                f'<box><size>{w:.3f} {d:.3f} {WALL_HEIGHT}</size></box>', 'wall'))

    return ('<?xml version="1.0"?>\n'
            '<!-- GENERATED by tools/generate_campus_world.py - edit the layout there, not here. -->\n'
            '<sdf version="1.6">\n'
            '  <model name="esda_campus">\n'
            '    <static>true</static>\n'
            + ''.join(parts) +
            '  </model>\n'
            '</sdf>\n')


def main():
    errors = validate()
    if errors:
        print('Layout check FAILED:')
        for e in errors:
            print(f'  - {e}')
        return 1

    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, 'w', newline='\n') as f:
        f.write(build_sdf())

    nearest = min(
        min(point_rect_dist(px, py, TRACK) for px, py in poly)
        for _n, _c, poly in footprints(0.0))
    print(f'Wrote {os.path.relpath(OUT_PATH, REPO_ROOT)}')
    print(f'  {len(BOXES)} boxes + {len(CYLINDERS)} cylinders, {len(PAVEMENT)} pavement patches')
    print(f'  nearest building to the track: {nearest:.1f} m')
    return 0


if __name__ == '__main__':
    sys.exit(main())
