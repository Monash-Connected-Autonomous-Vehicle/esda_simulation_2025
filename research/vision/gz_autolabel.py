#!/usr/bin/env python3
"""
gz_autolabel.py -- labelled frames from the REAL Gazebo renderer, for the
orange_igvc track that worlds/igvc*.sdf use.

synth_gen.py draws its own scenes, so it never shows the model ogre2's
shadows, lighting, round barrels or the black barricades. autolabel.py fixes
that for make_worlds.py worlds, using odometry poses. This does the same for
the orange_igvc track, and exactly:

  * Static cameras at chosen poses (no odometry drift), each paired with a
    depth camera at the same pose and intrinsics.
  * Every pixel is back-projected with its depth to a world point.
      - on the ground (z < GROUND_Z) inside the track mesh, where ground.png
        is white paint                          -> lane
      - inside a barrel's cylinder (from model.sdf) -> cone
    Shadows are in the image but never in the label, which is the lesson
    synth_gen cannot teach.
  * Masks become YOLOv8-seg polygons (class order lane, cone, drivable, as
    synth_gen.py writes them, so the two datasets mix).

Cameras match the sim ZED (640x480, hfov 1.089) on either mount; see
camera_front.yaml / camera_pole_top.yaml.

    # render N poses per run (needs Gazebo + ros_gz_bridge, no robot)
    python3 gz_autolabel.py render --world igvc.sdf --mount front --n 30 \\
        --seed 1 --out ~/esda_capture/gz_igvc
    # label everything rendered so far, with overlays to check by eye
    python3 gz_autolabel.py label --capture ~/esda_capture/gz_igvc \\
        --out ~/esda_datasets/lane_seg_gz --overlay

ALWAYS look at <out>/overlay before training: if the paint does not sit on
the paint, the texture mapping below is wrong and the dataset is poison.
"""
import argparse
import glob
import json
import math
import os
import random
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
WORLDS = os.path.join(REPO, 'src', 'esda_simulation_2025', 'worlds')
TRACK = os.path.join(WORLDS, 'models', 'orange_igvc')

# orange_igvc is included at this world pose by every igvc*.sdf.
TRACK_ORIGIN = (0.0, 16.3)
# ground.dae: a 43 x 37 m quad, uv (0,0) at (-21.5,-18.5), (1,1) at (21.5,18.5).
GROUND_HALF = (21.5, 18.5)
# construction_barrel.dae is in inches: radius 20.08 in, height 61.03 in.
BARREL_RADIUS = 0.51
BARREL_HEIGHT = 1.55
# The warning lamp on top reaches about 0.3 m higher, inside a smaller radius.
LAMP_RADIUS = 0.3
LAMP_TOP = 1.95

W, H = 640, 480
HFOV = 1.089
FX = (W / 2) / math.tan(HFOV / 2)
MOUNTS = {                       # height above ground (m), pitch down (rad)
    'front': (0.4775, 0.0),
    'pole_top': (1.4625, 0.2618),
}

GROUND_Z = 0.03                  # world z below which a point is "ground"
PAINT_GRAY = 200                 # ground.png gray above which it is paint
MIN_LANE_PX = 150                # keep a frame only if it shows this much ...
MIN_CONE_PX = 400                # ... lane or cone
MIN_POLY_AREA = 30               # px; smaller blobs are dropped


# ---------------------------------------------------------------------------
# World geometry
# ---------------------------------------------------------------------------

def load_track():
    """(barrels [(x, y)], keepouts [(x, y, r)]) in WORLD coordinates."""
    root = ET.parse(os.path.join(TRACK, 'model.sdf')).getroot()
    ox, oy = TRACK_ORIGIN
    barrels, keepouts = [], []
    for link in root.iter('link'):
        pose = [float(v) for v in (link.findtext('pose') or '0 0 0 0 0 0').split()]
        uris = [u.text or '' for u in link.iter('uri')]
        x, y = pose[0] + ox, pose[1] + oy
        if any('construction_barrel' in u for u in uris):
            barrels.append((x, y))
        elif link.get('name') != 'ground':
            keepouts.append((x, y, 2.5))       # barricades and ramp
    return barrels, keepouts


def load_paint():
    """Boolean paint mask of ground.png, row 0 = +y edge."""
    tex = cv2.imread(os.path.join(TRACK, 'materials', 'textures', 'ground.png'),
                     cv2.IMREAD_GRAYSCALE)
    return tex >= PAINT_GRAY


def paint_at(paint, wx, wy):
    """Whether world points (arrays) fall on painted lane."""
    th, tw = paint.shape
    mx = wx - TRACK_ORIGIN[0]
    my = wy - TRACK_ORIGIN[1]
    u = (mx + GROUND_HALF[0]) / (2 * GROUND_HALF[0])
    v = (my + GROUND_HALF[1]) / (2 * GROUND_HALF[1])
    inside = (u >= 0) & (u < 1) & (v >= 0) & (v < 1)
    col = np.clip((u * tw).astype(int), 0, tw - 1)
    row = np.clip(((1.0 - v) * th).astype(int), 0, th - 1)
    return inside & paint[row, col]


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def sample_poses(n, mount, rng, barrels, keepouts, paint):
    """Camera poses on the track, looking roughly along nearby paint."""
    z, pitch = MOUNTS[mount]
    ox, oy = TRACK_ORIGIN
    poses = []
    tries = 0
    while len(poses) < n and tries < n * 400:
        tries += 1
        x = ox + rng.uniform(-GROUND_HALF[0] + 1.5, GROUND_HALF[0] - 1.5)
        y = oy + rng.uniform(-GROUND_HALF[1] + 1.5, GROUND_HALF[1] - 1.5)
        if any(math.hypot(x - bx, y - by) < 1.2 for bx, by in barrels):
            continue
        if any(math.hypot(x - kx, y - ky) < r for kx, ky, r in keepouts):
            continue
        # Want paint 1-4 m away, i.e. standing in a lane, not mid-field.
        ang = np.linspace(0, 2 * math.pi, 24, endpoint=False)
        dists = np.linspace(0.8, 4.0, 9)
        px = x + np.outer(dists, np.cos(ang))
        py = y + np.outer(dists, np.sin(ang))
        hit = paint_at(paint, px, py)
        if not hit.any():
            continue
        # Look along the paint: the direction perpendicular to the nearest
        # paint bearing, either way, plus noise.
        d_idx, a_idx = np.argwhere(hit)[0]
        yaw = ang[a_idx] + rng.choice([-1, 1]) * math.pi / 2 + rng.gauss(0, 0.35)
        poses.append((x, y, z, pitch, yaw))
    return poses


def build_world(world_file, poses, tag):
    src = open(os.path.join(WORLDS, world_file)).read()
    models = []
    for i, (x, y, z, pitch, yaw) in enumerate(poses):
        for kind, typ in (('rgb', 'camera'), ('depth', 'depth_camera')):
            fmt = '' if typ == 'camera' else '<format>R_FLOAT32</format>'
            models.append(f'''
    <model name="cam_{tag}_{i}_{kind}"><static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.4f} 0 {pitch:.4f} {yaw:.4f}</pose>
      <link name="l"><sensor name="s" type="{typ}">
        <topic>gzl/{tag}/c{i}/{kind}</topic><update_rate>1</update_rate><always_on>1</always_on>
        <camera><horizontal_fov>{HFOV}</horizontal_fov>
          <image><width>{W}</width><height>{H}</height>{fmt}</image>
          <clip><near>0.05</near><far>60</far></clip></camera>
      </sensor></link></model>''')
    return src.replace('  </world>', ''.join(models) + '\n  </world>')


GRABBER = r'''
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
out, n, tag = sys.argv[1], int(sys.argv[2]), sys.argv[3]
rclpy.init(); node = Node('gzl_grab'); got = {}
def cb(msg, key):
    if key in got: return
    if key[1] == 'depth':
        a = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
    else:
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[..., ::-1].copy()
    got[key] = a
for i in range(n):
    for k in ('rgb', 'depth'):
        node.create_subscription(Image, f'/gzl/{tag}/c{i}/{k}', lambda m, key=(i, k): cb(m, key), 2)
t = time.time()
while len(got) < 2 * n and time.time() - t < 60:
    rclpy.spin_once(node, timeout_sec=0.2)
import cv2
for (i, k), a in got.items():
    if k == 'rgb': cv2.imwrite(f'{out}/{tag}_{i:03d}.png', a)
    else: np.save(f'{out}/{tag}_{i:03d}_depth.npy', a)
print(len(got), 'of', 2 * n)
'''


def cmd_render(args):
    out = os.path.expanduser(args.out)
    os.makedirs(out, exist_ok=True)
    rng = random.Random(args.seed)
    barrels, keepouts = load_track()
    paint = load_paint()
    poses = sample_poses(args.n, args.mount, rng, barrels, keepouts, paint)
    tag = f'{os.path.splitext(args.world)[0]}_{args.mount}_{args.seed}'
    world_path = os.path.join(out, f'_{tag}.sdf')
    with open(world_path, 'w') as f:
        f.write(build_world(args.world, poses, tag))
    grab_path = os.path.join(out, '_grab.py')
    with open(grab_path, 'w') as f:
        f.write(GRABBER)

    env = dict(os.environ)
    part = f'gzl_{os.getpid()}'
    env.update(IGN_PARTITION=part, GZ_PARTITION=part,
               ROS_DOMAIN_ID=str(80 + args.seed % 20),
               IGN_GAZEBO_RESOURCE_PATH=f'{WORLDS}:{os.path.join(WORLDS, "models")}')
    bridge_args = []
    for i in range(len(poses)):
        for k in ('rgb', 'depth'):
            bridge_args.append(f'/gzl/{tag}/c{i}/{k}@sensor_msgs/msg/Image[ignition.msgs.Image')
    gz = subprocess.Popen(['ign', 'gazebo', '-s', '-r', '-v', '1', '--headless-rendering', world_path],
                          env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + bridge_args,
                              env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(args.warmup)
        res = subprocess.run([sys.executable, grab_path, out, str(len(poses)), tag],
                             env=env, capture_output=True, text=True, timeout=120)
        print(f'[render] {tag}: {res.stdout.strip()} {res.stderr.strip()[-300:]}')
    finally:
        for p in (bridge, gz):
            p.terminate()
        for p in (bridge, gz):
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
    with open(os.path.join(out, f'{tag}_poses.json'), 'w') as f:
        json.dump({'world': args.world, 'mount': args.mount, 'poses': poses}, f)


# ---------------------------------------------------------------------------
# Labelling
# ---------------------------------------------------------------------------

def world_points(depth, pose):
    """(X, Y, Z) world coordinates for every pixel, NaN where no return."""
    x, y, z, pitch, yaw = pose
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    d = depth.astype(np.float64)
    d[~np.isfinite(d) | (d <= 0)] = np.nan
    # Camera body frame: x forward (depth), y left, z up.
    cx_, cy_ = (W - 1) / 2.0, (H - 1) / 2.0
    bx = d
    by = -(u - cx_) / FX * d
    bz = -(v - cy_) / FX * d
    # Rotate by pitch (about y, positive = nose down) then yaw (about z).
    cp, sp = math.cos(pitch), math.sin(pitch)
    x1 = cp * bx + sp * bz
    z1 = -sp * bx + cp * bz
    cyw, syw = math.cos(yaw), math.sin(yaw)
    wx = x + cyw * x1 - syw * by
    wy = y + syw * x1 + cyw * by
    wz = z + z1
    return wx, wy, wz


def label_frame(depth, pose, paint, barrels):
    wx, wy, wz = world_points(depth, pose)
    valid = np.isfinite(wz)
    ground = valid & (wz < GROUND_Z)
    lane = ground & paint_at(paint, np.nan_to_num(wx), np.nan_to_num(wy))
    cone = np.zeros_like(lane)
    for bx, by in barrels:
        r = np.hypot(wx - bx, wy - by)
        body = valid & (r < BARREL_RADIUS + 0.04) & (wz > -0.05) & (wz < BARREL_HEIGHT + 0.05)
        lamp = valid & (r < LAMP_RADIUS) & (wz >= BARREL_HEIGHT) & (wz < LAMP_TOP)
        cone |= body | lamp
    lane &= ~cone
    k = np.ones((3, 3), np.uint8)
    lane = cv2.morphologyEx(lane.astype(np.uint8), cv2.MORPH_CLOSE, k) > 0
    cone = cv2.morphologyEx(cone.astype(np.uint8), cv2.MORPH_OPEN, k) > 0
    return lane, cone


def mask_polygons(mask):
    cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                             cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cs:
        if cv2.contourArea(c) < MIN_POLY_AREA or len(c) < 3:
            continue
        c = cv2.approxPolyDP(c, 1.0, True).reshape(-1, 2)
        if len(c) >= 3:
            out.append(c)
    return out


def cmd_label(args):
    cap = os.path.expanduser(args.capture)
    out = os.path.expanduser(args.out)
    barrels, _ = load_track()
    paint = load_paint()
    poses = {}
    for pj in glob.glob(os.path.join(cap, '*_poses.json')):
        tag = os.path.basename(pj)[:-len('_poses.json')]
        poses[tag] = json.load(open(pj))['poses']
    rows = []
    for png in sorted(glob.glob(os.path.join(cap, '*.png'))):
        stem = os.path.splitext(os.path.basename(png))[0]
        tag, idx = stem.rsplit('_', 1)
        dpath = os.path.join(cap, stem + '_depth.npy')
        if tag not in poses or not os.path.exists(dpath):
            continue
        rows.append((png, dpath, poses[tag][int(idx)], stem))
    rng = random.Random(0)
    rng.shuffle(rows)
    n_val = int(len(rows) * args.val_frac)
    for split in ('train', 'val'):
        os.makedirs(os.path.join(out, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(out, 'labels', split), exist_ok=True)
    if args.overlay:
        os.makedirs(os.path.join(out, 'overlay'), exist_ok=True)
    kept = {'train': 0, 'val': 0}
    skipped = 0
    for k, (png, dpath, pose, stem) in enumerate(rows):
        split = 'val' if k < n_val else 'train'
        img = cv2.imread(png)
        depth = np.load(dpath)
        lane, cone = label_frame(depth, pose, paint, barrels)
        if lane.sum() < MIN_LANE_PX and cone.sum() < MIN_CONE_PX:
            skipped += 1
            continue
        lines = []
        for cls, m in ((0, lane), (1, cone)):
            for poly in mask_polygons(m):
                pts = ' '.join(f'{px / W:.5f} {py / H:.5f}' for px, py in poly)
                lines.append(f'{cls} {pts}')
        cv2.imwrite(os.path.join(out, 'images', split, stem + '.jpg'), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(os.path.join(out, 'labels', split, stem + '.txt'), 'w') as f:
            f.write('\n'.join(lines) + ('\n' if lines else ''))
        kept[split] += 1
        if args.overlay and sum(kept.values()) % args.overlay_every == 0:
            ov = img.copy()
            ov[lane] = (0.4 * ov[lane] + 0.6 * np.array([0, 255, 0])).astype(np.uint8)
            ov[cone] = (0.4 * ov[cone] + 0.6 * np.array([255, 0, 255])).astype(np.uint8)
            cv2.imwrite(os.path.join(out, 'overlay', stem + '.jpg'), ov)
    with open(os.path.join(out, 'data.yaml'), 'w') as f:
        f.write(f'# Generated by gz_autolabel.py\npath: {out}\n'
                'train: images/train\nval: images/val\n\n'
                'names:\n  0: lane\n  1: cone\n  2: drivable\n')
    print(f'[label] train {kept["train"]}, val {kept["val"]}, '
          f'skipped {skipped} (too little lane/cone) -> {out}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('render')
    r.add_argument('--world', default='igvc.sdf')
    r.add_argument('--mount', choices=sorted(MOUNTS), default='front')
    r.add_argument('--n', type=int, default=30)
    r.add_argument('--seed', type=int, default=1)
    r.add_argument('--warmup', type=float, default=14.0)
    r.add_argument('--out', default='~/esda_capture/gz_igvc')
    lab = sub.add_parser('label')
    lab.add_argument('--capture', default='~/esda_capture/gz_igvc')
    lab.add_argument('--out', default='~/esda_datasets/lane_seg_gz')
    lab.add_argument('--val-frac', type=float, default=0.15)
    lab.add_argument('--overlay', action='store_true')
    lab.add_argument('--overlay-every', type=int, default=10)
    args = ap.parse_args()
    {'render': cmd_render, 'label': cmd_label}[args.cmd](args)


if __name__ == '__main__':
    main()
