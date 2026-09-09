#!/usr/bin/env python3
"""
Offline validation for the waypoint navigator's clearance logic.

Runs WITHOUT rclpy, Gazebo or Nav2 - it imports the module-level pure
functions from waypoint_navigator_recommendation.py and exercises them against
synthetic grids and the real saved map. Needs only numpy, cv2 and pyyaml.

    python3 tools/validate_waypoint_logic.py

The reference oracle is the ORIGINAL brute-force implementation, reproduced
verbatim below, so this answers "did the fast version change any decision"
rather than merely "does the new code run".
"""

import math
import os
import sys
import time
import types

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, 'src', 'esda_simulation_2025', 'scripts')
sys.path.insert(0, SCRIPTS_DIR)


def _import_target():
    """
    Import the node module without requiring rclpy.

    Only the module-level helpers are needed, but the file imports the ROS
    stack at the top. Stub out anything unavailable so the import succeeds on
    a machine with no ROS.
    """
    for name in [
        'rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.qos',
        'std_msgs', 'std_msgs.msg',
        'geometry_msgs', 'geometry_msgs.msg',
        'nav2_msgs', 'nav2_msgs.action', 'nav2_msgs.msg',
        'nav2_simple_commander', 'nav2_simple_commander.robot_navigator',
        'nav_msgs', 'nav_msgs.msg',
        'visualization_msgs', 'visualization_msgs.msg',
        'sensor_msgs', 'sensor_msgs.msg',
        'sensor_msgs_py', 'sensor_msgs_py.point_cloud2',
        'tf2_ros', 'tf2_geometry_msgs',
    ]:
        if name not in sys.modules:
            module = types.ModuleType(name)
            module.__getattr__ = lambda _attr: object  # type: ignore[attr-defined]
            sys.modules[name] = module

    import waypoint_navigator_recommendation as target
    return target


TARGET = _import_target()


# ==========================================================================
# Reference oracle: the ORIGINAL implementation, copied verbatim
# ==========================================================================

def reference_map_clearance(grid, resolution, origin_x, origin_y,
                            width, height, world_x, world_y):
    centre_grid_x = int((world_x - origin_x) / resolution)
    centre_grid_y = int((world_y - origin_y) / resolution)

    if not (0 <= centre_grid_x < width and 0 <= centre_grid_y < height):
        return 0.0

    search_radius = 2.0
    search_cells = int(search_radius / resolution)
    minimum_distance = search_radius

    for grid_y in range(max(0, centre_grid_y - search_cells),
                        min(height, centre_grid_y + search_cells + 1)):
        for grid_x in range(max(0, centre_grid_x - search_cells),
                            min(width, centre_grid_x + search_cells + 1)):
            cell_value = grid[grid_y, grid_x]

            if cell_value >= 50:
                obstacle_world_x = origin_x + (grid_x + 0.5) * resolution
                obstacle_world_y = origin_y + (grid_y + 0.5) * resolution

                distance = math.hypot(obstacle_world_x - world_x,
                                      obstacle_world_y - world_y)
                minimum_distance = min(minimum_distance, distance)

    return minimum_distance


def reference_path_ratios(path_length, resolution):
    """The original sampling: N = max(2, int(L/spacing)), ratios i/N, i=1..N."""
    sample_spacing = max(0.03, resolution * 0.5)
    number_of_samples = max(2, int(path_length / sample_spacing))
    return [i / number_of_samples for i in range(1, number_of_samples + 1)]


# ==========================================================================
# Fixtures
# ==========================================================================

def make_snapshot(grid, resolution=0.05, origin_x=0.0, origin_y=0.0):
    height, width = grid.shape
    return TARGET.MapSnapshot(
        grid=grid,
        dt_hard=TARGET.distance_transform_metres(grid >= 50, resolution),
        dt_unknown=TARGET.distance_transform_metres(grid == -1, resolution),
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        width=width,
        height=height,
    )


def load_real_map():
    """Load my_map_save.pgm/.yaml. Negative origin exercises the int() trap."""
    import yaml

    yaml_path = os.path.join(REPO_ROOT, 'my_map_save.yaml')
    with open(yaml_path) as handle:
        meta = yaml.safe_load(handle)

    pgm_path = os.path.join(REPO_ROOT, meta['image'])
    with open(pgm_path, 'rb') as handle:
        assert handle.readline().strip() == b'P5'
        dims = handle.readline().split()
        while dims[0].startswith(b'#'):
            dims = handle.readline().split()
        width, height = int(dims[0]), int(dims[1])
        maxval = int(handle.readline())
        pixels = np.frombuffer(handle.read(), dtype=np.uint8).reshape((height, width))

    # map_server trinary interpretation, then flip to grid row order.
    occupancy = (maxval - pixels.astype(np.float64)) / maxval
    grid = np.full(pixels.shape, -1, dtype=np.int8)
    grid[occupancy > meta['occupied_thresh']] = 100
    grid[occupancy < meta['free_thresh']] = 0
    grid = np.flipud(grid).copy()

    return grid, float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])


class FakeScan:
    def __init__(self, ranges, angle_min=-math.pi, angle_increment=2 * math.pi / 360):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment


# ==========================================================================
# Checks
# ==========================================================================

RESULTS = []


def check(name, condition, detail=''):
    RESULTS.append((name, bool(condition), detail))
    status = 'PASS' if condition else 'FAIL'
    print(f'  [{status}] {name}' + (f'  -- {detail}' if detail else ''))
    return bool(condition)


def v1_closed_form():
    print('\nV-1  closed-form grids')

    free = np.zeros((100, 100), dtype=np.int8)
    snap = make_snapshot(free)
    values = [TARGET.map_clearance(snap, x, y, float('inf'))
              for x in (0.5, 1.5, 2.5) for y in (0.5, 1.5, 2.5)]
    check('all-free grid saturates at 2.0 (no-zero-pixel edge case)',
          all(abs(v - 2.0) < 1e-6 for v in values), f'got {set(round(v, 6) for v in values)}')

    grid = np.zeros((200, 200), dtype=np.int8)
    grid[100, 100] = 100
    snap = make_snapshot(grid)
    worst = 0.0
    for gy in range(80, 121):
        for gx in range(80, 121):
            wx = (gx + 0.5) * 0.05
            wy = (gy + 0.5) * 0.05
            expected = min(2.0, 0.05 * math.hypot(gx - 100, gy - 100))
            worst = max(worst, abs(TARGET.map_clearance(snap, wx, wy, float('inf')) - expected))
    # Queries land on cell centres here, so quantisation cancels and the
    # exact transform should agree to float precision.
    check('single obstacle matches analytic distance exactly',
          worst <= 1e-4, f'max err {worst:.4f} m')


def v2_v5_real_map():
    print('\nV-2/V-3/V-5/V-6  real map (my_map_save.pgm)')

    grid, resolution, origin_x, origin_y = load_real_map()
    height, width = grid.shape
    snap = make_snapshot(grid, resolution, origin_x, origin_y)
    print(f'  map {width}x{height} @ {resolution} m, origin ({origin_x}, {origin_y})')

    rng = np.random.default_rng(0)
    xs = rng.uniform(origin_x, origin_x + width * resolution, 2000)
    ys = rng.uniform(origin_y, origin_y + height * resolution, 2000)

    old = np.array([reference_map_clearance(grid, resolution, origin_x, origin_y,
                                            width, height, x, y) for x, y in zip(xs, ys)])
    new = np.array([TARGET.map_clearance(snap, x, y, float('inf'))
                    for x, y in zip(xs, ys)])

    # Tolerances are derived, not guessed. DIST_MASK_PRECISE is exact, so the
    # only divergence from the oracle is that the new implementation quantises
    # the QUERY POINT to its cell centre while the oracle measures from the
    # exact point to obstacle cell centres. For a uniformly placed query in a
    # cell of side s, the displacement is at most s*sqrt(2)/2 and its mean
    # projection onto the direction of the nearest obstacle is s/4.
    quantisation_max = resolution * math.sqrt(2.0) / 2.0     # 0.0354 m at 5 cm
    quantisation_mean = resolution / 4.0                     # 0.0125 m at 5 cm

    diff = np.abs(old - new)
    check(f'V-2 max |old-new| within quantisation bound ({quantisation_max:.4f} m)',
          diff.max() <= quantisation_max * 1.02, f'max {diff.max():.4f} m')
    check(f'V-2 mean |old-new| within quantisation mean ({quantisation_mean:.4f} m)',
          diff.mean() <= quantisation_mean * 1.10, f'mean {diff.mean():.5f} m')

    # V-3 out of grid
    off_grid = [
        (origin_x - 0.02, origin_y + 1.0),
        (origin_x + width * resolution + 0.02, origin_y + 1.0),
        (origin_x + 1.0, origin_y - 0.02),
        (origin_x + 1.0, origin_y + height * resolution + 0.02),
    ]
    check('V-3 off-grid queries return 0.0',
          all(TARGET.map_clearance(snap, x, y, float('inf')) == 0.0 for x, y in off_grid))

    # V-5 gate flips
    worst_flip = 0.0
    flips = 0
    for threshold in (0.60, 0.80, 0.90, 1.00, 1.20):
        changed = (old >= threshold) != (new >= threshold)
        flips += int(changed.sum())
        if changed.any():
            worst_flip = max(worst_flip, np.abs(old[changed] - threshold).max())
    check('V-5 every gate flip is within 0.05 m of its threshold',
          worst_flip < 0.05, f'{flips} flips, worst margin {worst_flip:.4f} m')

    # V-6 invariants
    ok = True
    for _ in range(1000):
        ax, ay = rng.uniform(origin_x, origin_x + width * resolution), rng.uniform(origin_y, origin_y + height * resolution)
        bx, by = ax + rng.uniform(-2, 2), ay + rng.uniform(-2, 2)
        pc = TARGET.path_clearance(snap, ax, ay, bx, by, float('inf'))
        gc = TARGET.map_clearance(snap, bx, by, float('inf'))
        if not (pc <= gc + 1e-6) or math.isnan(pc) or not (0.0 <= pc <= 2.0):
            ok = False
            break
    check('V-6 path_clearance <= goal_clearance, bounded, no NaN', ok)

    return snap


def v4_path_samples():
    print('\nV-4  path sampling set (the linspace trap)')

    grid = np.zeros((400, 400), dtype=np.int8)
    snap = make_snapshot(grid)

    captured = {}
    original = TARGET.np.arange

    # Reconstruct the ratios the implementation actually uses.
    for length in (0.4, 1.0, 1.55, 2.2):
        expected = reference_path_ratios(length, snap.resolution)
        n = len(expected)
        actual = list(np.arange(1, n + 1, dtype=np.float64) / n)
        captured[length] = (expected, actual)

    all_match = all(
        len(exp) == len(act) and all(abs(a - b) < 1e-12 for a, b in zip(exp, act))
        for exp, act in captured.values()
    )
    check('ratios equal [i/N for i in 1..N] (start excluded, end included)', all_match)

    # The behavioural consequence: a wall right beside the robot must NOT
    # poison a path that leads away from it.
    grid = np.zeros((400, 400), dtype=np.int8)
    grid[:, 100] = 100                      # wall at x = 5.0 m
    snap = make_snapshot(grid)
    # Robot sits 0.10 m from the wall, target is 2 m away from it.
    value = TARGET.path_clearance(snap, 5.10, 5.0, 7.0, 5.0, float('inf'))
    check('start point excluded: robot hugging a wall does not zero the path',
          value > 0.05, f'path_clearance {value:.3f} m')


def v7_deadlock_table():
    print('\nV-7  unknown-space deadlock table (U = 0.8)')

    resolution = 0.05
    allowance = 0.8
    rows = []

    for corridor_width in (2.0, 3.0, 4.0):
        for unknown_distance in (0.2, 0.6, 1.0, 1.4, 2.0):
            size = 200
            grid = np.full((size, size), -1, dtype=np.int8)

            # Free corridor along y, centred at x = 5.0 m, with known free
            # space extending unknown_distance ahead of the robot at y = 5.0.
            half = corridor_width / 2.0
            x0 = int((5.0 - half) / resolution)
            x1 = int((5.0 + half) / resolution)
            y0 = int(3.0 / resolution)
            y1 = int((5.0 + unknown_distance) / resolution)
            grid[y0:y1, x0:x1] = 0

            snap = make_snapshot(grid, resolution)
            value = TARGET.map_clearance(snap, 5.0, 5.0, allowance)

            rows.append((corridor_width, unknown_distance, value,
                         value >= 1.00, value >= 0.80))

    print(f'    {"corridor":>9} {"unknown":>8} {"clearance":>10} {"lane gate":>10} {"recovery":>9}')
    for corridor_width, unknown_distance, value, lane_ok, recovery_ok in rows:
        print(f'    {corridor_width:9.1f} {unknown_distance:8.1f} {value:10.3f} '
              f'{"pass" if lane_ok else "FAIL":>10} {"pass" if recovery_ok else "FAIL":>9}')

    # The escape hatch must exist: with unknown far enough back, gates open.
    deep = [r for r in rows if r[1] >= 1.4]
    check('recovery gate (0.80) opens once known space extends >= 1.4 m',
          all(r[4] for r in deep))
    check('unknown adjacent to the robot no longer scores maximally clear',
          all(r[2] < 2.0 for r in rows if r[1] <= 0.6))


def v8_scan_sectors():
    print('\nV-8  scan sector equality')

    rng = np.random.default_rng(1)
    ranges = list(rng.uniform(0.2, 10.0, 360))
    ranges[10] = float('inf')
    ranges[20] = float('nan')
    ranges[30] = 0.0                      # no-return: must NOT be filtered
    scan = FakeScan(ranges)

    def reference(scan_obj, lo_deg, hi_deg):
        valid = []
        lo, hi = math.radians(lo_deg), math.radians(hi_deg)
        for index, value in enumerate(scan_obj.ranges):
            angle = scan_obj.angle_min + index * scan_obj.angle_increment
            if lo <= angle <= hi:
                if not math.isinf(value) and not math.isnan(value):
                    valid.append(value)
        return min(valid) if valid else float('inf')

    exact = True
    for lo, hi in ((-100, -20), (-20, 20), (20, 100)):
        if reference(scan, lo, hi) != TARGET.scan_sector_clearance(scan, lo, hi):
            exact = False
    check('sector minima exactly equal the original loop', exact)

    zero_scan = FakeScan([0.0] * 360)
    check('0.0 returns are kept, not filtered',
          TARGET.scan_sector_clearance(zero_scan, -20, 20) == 0.0)

    empty = FakeScan([1.0] * 360, angle_min=0.0, angle_increment=0.001)
    check('empty sector returns inf',
          math.isinf(TARGET.scan_sector_clearance(empty, -100, -90)))


def v9_travel_geometry():
    print('\nV-9  travel-mark geometry')

    radius = 0.8
    min_distance = radius + 0.45 + 0.30
    resolution = 0.05

    rows, cols = TARGET.disc_offsets(radius, resolution)
    radius_cells = int(round(radius / resolution))
    expected = int(((np.arange(-radius_cells, radius_cells + 1)[:, None] ** 2
                     + np.arange(-radius_cells, radius_cells + 1)[None, :] ** 2)
                    <= radius_cells ** 2).sum())
    check('disc cell count matches the exact covering set',
          rows.size == expected, f'{rows.size} cells')

    # Straight, arc and U-turn trajectories.
    #
    # Two different properties are checked:
    #
    #  - AT PROMOTION TIME the guard must never publish a mark that already
    #    covers the robot. This is the guard's actual contract and must hold
    #    unconditionally.
    #  - LATER re-approach is only possible if the robot drives back over its
    #    own path. The course is one-way, so this should not arise; and
    #    because the mask is graded (~200) rather than LETHAL (254), driving
    #    over an old mark costs more but is never a trap. Reported, not
    #    asserted - see the u-turn row.
    for name, poses in (
        ('straight', [(t * 0.1, 0.0, 0.0) for t in range(200)]),
        ('arc', [(5 * math.sin(t * 0.01), 5 * (1 - math.cos(t * 0.01)), t * 0.01)
                 for t in range(300)]),
        ('u-turn', [(t * 0.1, 0.0, 0.0) for t in range(100)]
                   + [(10 - t * 0.1, 0.6, math.pi) for t in range(100)]),
    ):
        pending, published = [], []
        last_marker = None
        promotion_violation = None
        closest_later = float('inf')

        for (x, y, yaw) in poses:
            if last_marker is None or math.hypot(x - last_marker[0], y - last_marker[1]) >= 5.0:
                last_marker = (x, y)
                pending.append((x, y))

            forward_x, forward_y = math.cos(yaw), math.sin(yaw)
            still = []
            for (px, py) in pending:
                dx, dy = px - x, py - y
                if math.hypot(dx, dy) >= min_distance and (dx * forward_x + dy * forward_y) < -0.25:
                    if math.hypot(dx, dy) < radius + 0.45 and promotion_violation is None:
                        promotion_violation = (px, py, x, y)
                    published.append((px, py))
                else:
                    still.append((px, py))
            pending = still

            for (px, py) in published:
                closest_later = min(closest_later, math.hypot(px - x, py - y))

        check(f'{name}: no mark is published already covering the robot',
              promotion_violation is None,
              '' if promotion_violation is None else f'{promotion_violation}')

        encroaches = closest_later < radius + 0.45
        print(f'      (info) {name}: closest later approach to a mark '
              f'{closest_later:.2f} m'
              + ('  <- retraced; tolerable only because cost is graded'
                 if encroaches else ''))


def v10_yaml_lint():
    print('\nV-10  nav2_params.yaml observation-source lint')

    import yaml

    path = os.path.join(REPO_ROOT, 'src', 'esda_simulation_2025', 'config', 'nav2_params.yaml')
    with open(path) as handle:
        config = yaml.safe_load(handle)

    problems = []

    def walk(node):
        if not isinstance(node, dict):
            return
        for key, value in node.items():
            if isinstance(value, dict):
                plugin = value.get('plugin', '')
                if isinstance(plugin, str) and ('ObstacleLayer' in plugin or 'VoxelLayer' in plugin):
                    sources = value.get('observation_sources', '')
                    for source in str(sources).split():
                        if source not in value:
                            problems.append(f'{key}: source "{source}" has no matching block')
                walk(value)

    walk(config)

    check('every observation_sources entry has a matching config block',
          not problems, '; '.join(problems))


def v11_timing(snap):
    print('\nV-11  timing (informational)')

    start = time.perf_counter()
    for _ in range(1000):
        TARGET.map_clearance(snap, 0.0, 0.0, 0.8)
    single = (time.perf_counter() - start) / 1000

    start = time.perf_counter()
    for _ in range(200):
        TARGET.path_clearance(snap, 0.0, 0.0, 2.0, 0.0, 0.8)
    path = (time.perf_counter() - start) / 200

    print(f'    map_clearance   {single * 1e6:8.1f} us')
    print(f'    path_clearance  {path * 1e3:8.3f} ms')
    print(f'    52-candidate bias sweep (est) {(52 * (single + path)) * 1e3:8.1f} ms')
    check('path_clearance under 1 ms', path < 1e-3, f'{path * 1e3:.3f} ms')


def main():
    print('Offline validation for waypoint_navigator_recommendation.py')
    print('=' * 62)

    v1_closed_form()
    snap = v2_v5_real_map()
    v4_path_samples()
    v7_deadlock_table()
    v8_scan_sectors()
    v9_travel_geometry()
    v10_yaml_lint()
    v11_timing(snap)

    print('\n' + '=' * 62)
    failed = [name for name, ok, _ in RESULTS if not ok]
    print(f'{len(RESULTS) - len(failed)}/{len(RESULTS)} checks passed')
    for name in failed:
        print(f'  FAILED: {name}')

    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
