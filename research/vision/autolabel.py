#!/usr/bin/env python3
"""
autolabel.py -- turn REAL Gazebo frames into a labelled dataset, automatically.

THE PROBLEM THIS SOLVES

The model is trained entirely on synth_gen.py frames: our own lane geometry
drawn onto the sim's asphalt texture with the exact camera model. They are
correct and they are free, and the model scores well on them. Then it meets
the ogre2 renderer and goes blind on hard cast shadows, because the generator
never produced one. Measured, repeatedly: it holds +-0.2 m over 20 m while it
can see paint, and stops when it cannot.

The fix is real renderer frames in the training set. The reason that has not
happened is labelling: capture_frames.py writes jpgs and then somebody has to
draw polygons on a few thousand of them.

But in simulation the labels are not unknown. We know where the paint is -- it
IS the ground texture, and make_worlds.py put it there at known world
coordinates. We know where every barrel is, from the world SDF. And we know
where the camera was, from odometry. So the label can be computed rather than
drawn:

    for each image pixel -> its point on the ground   (synth_gen.pixel_to_ground)
    -> rotate/translate into world coordinates        (the recorded robot pose)
    -> look up the ground texture there               (w2p, as make_worlds wrote it)
    -> that pixel is lane paint, or it is not

An inverse warp, so there are no holes, and it is exact up to odometry drift.
Barrels are projected forward from their SDF positions as silhouettes.

WHAT THIS BUYS

The image contains the renderer's real shadows, real speckle, real lighting.
The label contains none of them -- a shadow is asphalt, because the texture
says so. That is precisely the lesson the model is missing, and it arrives on
thousands of frames for free.

ALWAYS LOOK AT THE OVERLAYS BEFORE TRAINING

    python3 autolabel.py --capture ~/esda_capture/igvc_oval_test --overlay

writes <out>/overlay/*.jpg with the computed masks drawn over the frames.
Auto-labels are only as good as the pose, and odometry drifts. Flick through
twenty of them. If the paint does not sit on the paint, the dataset is poison
and no amount of training fixes it.

USAGE
    python3 autolabel.py --capture ~/esda_capture/igvc_oval_test \\
                         --repo /mnt/c/work/felix/mcav/for_server \\
                         --out ~/esda_datasets/real_sim --overlay
"""
import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import re

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import synth_gen as sg                                           # noqa: E402

try:
    import cv2
    HAVE_CV = True
except ImportError:
    HAVE_CV = False

# The ground plane, exactly as make_worlds.py and the .dae agree on it:
#   mesh spans x [-21.5, 21.5], y [-18.5, 18.5]; uv 0..1 over that rectangle
#   texture is 1400 x 1200
# w2p below is make_worlds.w2p. If one changes, change both -- note the v
# flip, which is the one thing that is easy to get silently wrong and which
# would put every label in the wrong half of the course.
X_MIN, X_MAX = -21.5, 21.5
Y_MIN, Y_MAX = -18.5, 18.5
TEX_W, TEX_H = 1400, 1200

BARREL_DIAM = 0.58       # IGVC construction barrel, matches cone_radius 0.28
BARREL_HEIGHT = 0.90


def w2p(X, Y):
    """World metres -> texture pixel (u, v). Mirrors make_worlds.w2p."""
    u = (np.asarray(X, float) - X_MIN) / (X_MAX - X_MIN) * TEX_W
    v = (Y_MAX - np.asarray(Y, float)) / (Y_MAX - Y_MIN) * TEX_H
    return u, v


def paint_mask(tex_bgr, v_min=150, s_max=70):
    """Binary mask of the painted lines in the ground texture.

    The lines are near-white on grey asphalt, so bright-and-unsaturated picks
    them out. Deliberately NOT a plain brightness threshold: the asphalt has
    bright speckle, and the saturation term is what separates paint from a
    pale stone.
    """
    hsv = cv2.cvtColor(tex_bgr, cv2.COLOR_BGR2HSV)
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    return ((v >= v_min) & (s <= s_max)).astype(np.uint8)


def barrels_from_sdf(path):
    """[(x, y)] for every barrel in the world file.

    make_worlds.py writes each barrel as a <link name="barrelN"> inside the
    arena <model>, NOT as a model of its own. Searching <model> elements
    therefore matches the whole arena -- whose subtree does contain barrel
    meshes -- and yields exactly one "barrel" sitting at the arena origin.
    Innermost container wins: links first, models only if a world happens to
    use them instead.
    """
    root = ET.parse(path).getroot()

    def collect(tag):
        out = []
        for el in root.iter(tag):
            uris = " ".join(e.text or "" for e in el.iter("uri"))
            if "construction_barrel" not in uris:
                continue
            pose = el.find("pose")
            if pose is None or not pose.text:
                continue
            n = pose.text.split()
            if len(n) >= 2:
                out.append((float(n[0]), float(n[1])))
        return out

    return collect("link") or collect("model")


def label_frame(paint, barrels, pose, pitch_deg=0.0, pix=None,
                barrel_diam=BARREL_DIAM, barrel_height=BARREL_HEIGHT):
    """Masks for one frame. pose is (X, Y, yaw) of base_link in the world."""
    X, Y, valid = pix if pix is not None else sg.pixel_to_ground(pitch_deg)
    rx, ry, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)

    # base_link -> world, then world -> texture, then sample.
    wx = rx + c * X - s * Y
    wy = ry + s * X + c * Y
    u, v = w2p(wx, wy)
    iu = np.clip(np.round(u).astype(np.int32), 0, TEX_W - 1)
    iv = np.clip(np.round(v).astype(np.int32), 0, TEX_H - 1)
    inside = valid & (u >= 0) & (u < TEX_W) & (v >= 0) & (v < TEX_H)

    lane = (paint[iv, iu] > 0) & inside

    # Barrels: silhouette between the base ring and the top rim.
    cone = np.zeros((sg.H_IMG, sg.W), np.uint8)
    r = 0.5 * barrel_diam
    for (bx, by) in barrels:
        dx, dy = bx - rx, by - ry
        lx = c * dx + s * dy            # into base_link
        ly = -s * dx + c * dy
        if lx < 0.3 or lx > sg.MAX_X or abs(ly) > 0.75 * lx + 3.0:
            continue
        th = np.linspace(0, 2 * np.pi, 24)
        cxs, cys = bx + r * np.cos(th), by + r * np.sin(th)
        ddx, ddy = cxs - rx, cys - ry
        rxs = c * ddx + s * ddy
        rys = -s * ddx + c * ddy
        ok = rxs > 0.3
        if not ok.any():
            continue
        ub, vb = sg.ground_to_pixel(rxs[ok], rys[ok], pitch_deg)
        ut, vt = sg.ground_to_pixel_elevated(rxs[ok], rys[ok],
                                             barrel_height, pitch_deg)
        pts = np.concatenate([np.stack([ub, vb], 1),
                              np.stack([ut, vt], 1)[::-1]]).astype(np.int32)
        hull = cv2.convexHull(pts)
        cv2.fillConvexPoly(cone, hull, 1)
    cone = (cone > 0)
    lane &= ~cone                       # a barrel in front of paint wins

    # Drivable: ground you can reach from just ahead of the robot without
    # crossing a line or a barrel. A flood fill says that directly, and needs
    # no corridor polygon.
    free = (inside & ~lane & ~cone).astype(np.uint8)
    drivable = np.zeros_like(free)
    seed_v = sg.H_IMG - 12
    seeds = [(sg.W // 2, seed_v), (sg.W // 2 - 40, seed_v),
             (sg.W // 2 + 40, seed_v)]
    ff = free.copy()
    mask = np.zeros((sg.H_IMG + 2, sg.W + 2), np.uint8)
    for (su, sv) in seeds:
        if ff[sv, su]:
            cv2.floodFill(ff, mask, (su, sv), 2)
    drivable = (ff == 2)
    return {"lane": lane, "cone": cone, "drivable": drivable, "valid": inside}


def polys(masks, want_drivable=True):
    out = []
    out += sg.mask_to_polys(masks["lane"], sg.LANE)
    out += sg.mask_to_polys(masks["cone"], sg.CONE)
    if want_drivable:
        out += sg.mask_to_polys(masks["drivable"], sg.DRIVABLE, min_area=600)
    return out


def overlay(img, masks):
    o = img.copy()
    o[masks["drivable"]] = (0.6 * o[masks["drivable"]]
                            + 0.4 * np.array([0, 160, 0])).astype(np.uint8)
    o[masks["lane"]] = (0.25 * o[masks["lane"]]
                        + 0.75 * np.array([255, 0, 255])).astype(np.uint8)
    o[masks["cone"]] = (0.35 * o[masks["cone"]]
                        + 0.65 * np.array([0, 140, 255])).astype(np.uint8)
    return o


# ---------------------------------------------------------------------------
def chunked_split(names, val_frac, chunk=25):
    """Split into train/val by CONTIGUOUS BLOCKS, never at random.

    Frames arrive at 1-2 Hz from a robot moving at walking pace, so frame N
    and frame N+1 are nearly the same picture. A random split puts one in
    train and the other in val, the model scores beautifully, and the number
    means nothing. Holding out whole blocks of the run keeps val honest, the
    same reason the lap benchmark uses worlds from a different seed.
    """
    names = sorted(names)
    blocks = [names[i:i + chunk] for i in range(0, len(names), chunk)]
    n_val = max(1, int(round(len(blocks) * val_frac))) if blocks else 0
    step = max(1, len(blocks) // n_val) if n_val else 1
    val_idx = set(range(0, len(blocks), step))
    while len(val_idx) > n_val:
        val_idx.discard(max(val_idx))
    train, val = [], []
    for i, b in enumerate(blocks):
        (val if i in val_idx else train).extend(b)
    return train, val


GROUND_MESH_RE = re.compile(
    r"<uri>\s*(?:package://)?(\S*meshes/ground[A-Za-z0-9_]*\.dae)\s*</uri>")
DAE_IMG_RE = re.compile(r"<init_from>\s*(\S+\.(?:png|jpg))\s*</init_from>")
INCLUDE_RE = re.compile(r"<include>(.*?)</include>", re.S)
POSE_RE = re.compile(r"<pose>([^<]*)</pose>")
MODEL_URI_RE = re.compile(r"<uri>\s*(?:package://)?models/([A-Za-z0-9_]+)\s*</uri>")


def _texture_of_dae(dae):
    """The image a .dae declares. The mesh is authoritative, not its name."""
    if not os.path.exists(dae):
        raise SystemExit(f"ground mesh missing: {dae}")
    imgs = DAE_IMG_RE.findall(open(dae).read())
    if not imgs:
        raise SystemExit(f"no texture <init_from> in {dae}")
    tex = os.path.normpath(os.path.join(os.path.dirname(dae), imgs[0]))
    if not os.path.exists(tex):
        raise SystemExit(f"{dae} points at {imgs[0]}, which is missing")
    return tex


def ground_texture_for(sdf_path, repo):
    """The ground texture this world actually paints its lane lines on.

    THE WORLD NAME IS NOT THE TEXTURE NAME. igvc_oval_slow_test is the oval
    course at quarter speed and reuses ground_oval_test.dae. Deriving
    "ground_oval_slow_test.png" from the world name found no file, so job 109
    produced no labels, no data.yaml and no overlays -- after a ten-minute
    capture -- and the recall step printed "no images under .../val" where a
    number should have been. Every _slow and _crawl variant had the same hole.

    So follow the chain the simulator follows: the world .sdf names a ground
    mesh, and the .dae names its texture image. Nothing is guessed from a name.
    """
    pkg = os.path.join(repo, "esda_simulation_2025")
    sdf = open(sdf_path).read()

    direct = GROUND_MESH_RE.findall(sdf)
    if direct:
        return _texture_of_dae(os.path.join(pkg, direct[0]))

    # The stock world draws no ground of its own: it <include>s the
    # orange_igvc model, which does.
    for body in INCLUDE_RE.findall(sdf):
        m = MODEL_URI_RE.search(body)
        if not m:
            continue
        model = m.group(1)
        pose = POSE_RE.search(body)
        off = [float(v) for v in (pose.group(1).split() if pose else [])[:3]] or [0.0, 0.0, 0.0]
        if any(abs(v) > 1e-6 for v in off):
            raise SystemExit(
                f"{os.path.basename(sdf_path)} gets its ground from the "
                f"included model '{model}', placed at pose "
                f"{off[0]} {off[1]} {off[2]}.\n"
                "w2p() here mirrors make_worlds.w2p, which assumes the texture "
                "is centred on the world origin -- true for the generated "
                "arenas, false for this one. Scoring it would put every label "
                "off by that offset and read as a broken model rather than a "
                "broken transform. Score a generated arena (igvc_oval_test "
                "and friends), or teach w2p the include pose first.")
        sub = os.path.join(pkg, "models", model, "model.sdf")
        if not os.path.exists(sub):
            raise SystemExit(f"{sdf_path} includes '{model}', but {sub} is missing")
        meshes = GROUND_MESH_RE.findall(open(sub).read())
        if not meshes:
            raise SystemExit(f"no ground mesh in {sub}")
        return _texture_of_dae(os.path.join(pkg, meshes[0]))

    raise SystemExit(f"no ground mesh and no model <include> in {sdf_path} "
                     f"-- cannot tell which texture holds the paint")


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--capture", required=True,
                   help="dir of frame_*.jpg + frame_*.json from capture_labelled.py")
    p.add_argument("--repo", default=os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))
    p.add_argument("--out", default=os.path.expanduser("~/esda_datasets/real_sim"))
    p.add_argument("--world", default="", help="override the world in the json")
    p.add_argument("--overlay", action="store_true",
                   help="also write overlay/*.jpg -- LOOK AT THESE")
    p.add_argument("--overlay-every", type=int, default=10)
    p.add_argument("--val-frac", type=float, default=0.15)
    p.add_argument("--no-drivable", action="store_true")
    p.add_argument("--pitch", type=float, default=0.0)
    p.add_argument("--limit", type=int, default=0)
    a = p.parse_args()
    if not HAVE_CV:
        raise SystemExit("needs opencv: pip install opencv-python")

    frames = sorted(f for f in os.listdir(a.capture) if f.endswith(".jpg"))
    if a.limit:
        frames = frames[:a.limit]
    if not frames:
        raise SystemExit(f"no frames in {a.capture}")

    # Everything in one capture dir shares a world, so load its texture and
    # barrels once -- and cache the pixel->ground grid, which is the same for
    # every frame at a fixed pitch and is the slow part.
    first = json.load(open(os.path.join(a.capture,
                                        frames[0].replace(".jpg", ".json"))))
    world = a.world or first.get("world", "")
    sdf = os.path.join(a.repo, "esda_simulation_2025", "worlds", f"{world}.sdf")
    if not os.path.exists(sdf):
        raise SystemExit(f"missing {sdf} -- is --world '{world}' right?")
    tex = ground_texture_for(sdf, a.repo)

    paint = paint_mask(cv2.imread(tex, cv2.IMREAD_COLOR))
    barrels = barrels_from_sdf(sdf)
    pix = sg.pixel_to_ground(a.pitch)
    print(f"[autolabel] world={world}  {len(frames)} frames  "
          f"{len(barrels)} barrels  paint={paint.mean()*100:.1f}% of texture")

    tr, va = chunked_split([f[:-4] for f in frames], a.val_frac)
    split_of = {n: "train" for n in tr}
    split_of.update({n: "val" for n in va})
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(a.out, sub), exist_ok=True)
    if a.overlay:
        os.makedirs(os.path.join(a.out, "overlay"), exist_ok=True)

    kept = skipped = 0
    for i, fn in enumerate(frames):
        stem = fn[:-4]
        jf = os.path.join(a.capture, stem + ".json")
        if not os.path.exists(jf):
            skipped += 1
            continue
        meta = json.load(open(jf))
        img = cv2.imread(os.path.join(a.capture, fn), cv2.IMREAD_COLOR)
        if img is None:
            skipped += 1
            continue
        m = label_frame(paint, barrels,
                        (meta["x"], meta["y"], meta["yaw"]), a.pitch, pix)
        lines = polys(m, want_drivable=not a.no_drivable)
        if not lines:
            skipped += 1               # nothing visible: not a useful sample
            continue
        sp = split_of[stem]
        cv2.imwrite(os.path.join(a.out, "images", sp, stem + ".jpg"), img)
        with open(os.path.join(a.out, "labels", sp, stem + ".txt"), "w") as fh:
            fh.write("\n".join(lines) + "\n")
        if a.overlay and i % a.overlay_every == 0:
            cv2.imwrite(os.path.join(a.out, "overlay", stem + ".jpg"),
                        overlay(img, m))
        kept += 1

    with open(os.path.join(a.out, "data.yaml"), "w") as fh:
        fh.write(f"path: {os.path.abspath(a.out)}\n"
                 "train: images/train\nval: images/val\n\nnames:\n"
                 "  0: lane\n  1: cone\n  2: drivable\n")
    print(f"[autolabel] kept {kept}, skipped {skipped}  "
          f"(train {len(tr)}, val {len(va)})")
    print(f"[autolabel] dataset: {a.out}")
    if a.overlay:
        print(f"[autolabel] LOOK AT {os.path.join(a.out, 'overlay')} BEFORE "
              "TRAINING -- if the magenta does not sit on the painted lines, "
              "the pose is drifting and the labels are wrong.")


if __name__ == "__main__":
    main()
