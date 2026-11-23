#!/usr/bin/env python3
"""Build the IGVC course as a USD stage, from the Gazebo world it already has.

SOURCE OF TRUTH
    worlds/models/orange_igvc/model.sdf. Every pose is read from there rather
    than retyped, and igvc.sdf's include offset of (0, 16.3) is applied, so
    Isaac coordinates line up with the Gazebo sim, my_map_save.pgm, the
    waypoints in igvc_waypoints.yaml and the centreline that
    extract_centreline.py recovered.

WHAT IT BUILDS
    ground      a 43 x 37 m quad carrying ground.png, which is where the lane
                markings live -- they are painted into the texture, not
                modelled. A separate thin box provides the collider, because a
                textured visual mesh is the wrong thing to hand PhysX.
    barrels     34 cylinders, r 0.25 x 1.0 m. NOT the 189 KB DAE: the repo
                already approximates them this way in worlds/sim_world_ic.sdf,
                at the same coordinates, and a convex hull of that mesh would
                be both slower and less predictable.
    barricades  4 boxes, 3 x 0.5 x 3 m, standing at z 1.5
    ramps       2 slabs, 4 x 2 x 0.05 m, rolled 0.148 rad (8.5 deg) and
                meeting in a peak -- the obstacle the whole tilt filter exists
                for

    It also writes config/course_obstacles.yaml: the same obstacles as plain
    numbers, for scan_sim.py to raycast against without opening a USD stage.
    That file and course_meta.yaml are deliberately separate -- the centreline
    comes from the texture, the colliders come from the SDF, and conflating
    two different sources in one generated file invites one to be regenerated
    without the other.

A NOTE ON TRUSTING THIS
    Phase 1 was a long lesson in transforms that read correctly and render
    wrong. --snapshot renders the result; look at it before believing it.

USAGE
    C:\\Users\\Ishaan\\IsaacLab\\isaaclab.bat -p research\\isaac\\scripts\\build_course_usd.py
    ... --snapshot --enable_cameras
"""
import argparse
import math
import os
import shutil
import sys
import xml.dom.minidom as minidom

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_DIR = os.path.abspath(os.path.join(HERE, ".."))
REPO = os.path.abspath(os.path.join(ISAAC_DIR, "..", ".."))
PKG = os.path.join(REPO, "src", "esda_simulation_2025")
MODEL_DIR = os.path.join(PKG, "worlds", "models", "orange_igvc")
MODEL_SDF = os.path.join(MODEL_DIR, "model.sdf")
GROUND_PNG = os.path.join(MODEL_DIR, "materials", "textures", "ground.png")

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--sdf", default=MODEL_SDF)
parser.add_argument("--out-dir", default=os.path.join(ISAAC_DIR, "usd", "course"))
parser.add_argument("--name", default="igvc_course.usda")
parser.add_argument("--obstacles",
                    default=os.path.join(ISAAC_DIR, "config", "course_obstacles.yaml"))
parser.add_argument("--snapshot", action="store_true",
                    help="render the finished course; needs --enable_cameras")

from isaaclab.app import AppLauncher  # noqa: E402

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

# igvc.sdf: <include><pose>0 16.3 0</pose><uri>model://orange_igvc</uri>
MODEL_OFFSET = (0.0, 16.3, 0.0)

# meshes/ground.dae, as read by research/vision/make_worlds.py
ARENA_X, ARENA_Y = 43.0, 37.0

# worlds/sim_world_ic.sdf's own approximation of construction_barrel.dae
BARREL_RADIUS, BARREL_HEIGHT = 0.25, 1.0

GROUND_THICKNESS = 0.4


# ------------------------------------------------------------------- parse --
def _floats(text):
    return [float(v) for v in text.split()]


def read_pose(link):
    """SDF <pose>x y z roll pitch yaw</pose>, with the include offset applied."""
    for node in link.childNodes:
        if node.nodeType == node.ELEMENT_NODE and node.tagName == "pose":
            v = _floats(node.firstChild.data)
            while len(v) < 6:
                v.append(0.0)
            return (v[0] + MODEL_OFFSET[0], v[1] + MODEL_OFFSET[1],
                    v[2] + MODEL_OFFSET[2], v[3], v[4], v[5])
    return (MODEL_OFFSET[0], MODEL_OFFSET[1], MODEL_OFFSET[2], 0.0, 0.0, 0.0)


def box_size(link):
    for coll in link.getElementsByTagName("collision"):
        for box in coll.getElementsByTagName("box"):
            return _floats(box.getElementsByTagName("size")[0].firstChild.data)
    return None


def parse_world(path):
    doc = minidom.parse(path)
    out = {"barrels": [], "barricades": [], "ramps": []}
    for link in doc.getElementsByTagName("link"):
        name = link.getAttribute("name")
        pose = read_pose(link)
        if name.startswith("construction_barrel"):
            out["barrels"].append({"name": name, "pose": pose})
        elif name.startswith("barricade"):
            out["barricades"].append({"name": name, "pose": pose,
                                      "size": box_size(link) or [3.0, 0.5, 3.0]})
        elif name.startswith("ramp_part"):
            out["ramps"].append({"name": name, "pose": pose,
                                 "size": box_size(link) or [4.0, 2.0, 0.05]})
    return out


# ------------------------------------------------------------------ author --
def set_pose(prim, xyz, rpy=(0.0, 0.0, 0.0), scale=None):
    """Place a prim from an SDF pose.

    SDF rpy composes as Rz(yaw) Ry(pitch) Rx(roll), and USD's xformOp:rotateXYZ
    applies X then Y then Z -- the same composition. So the angles transfer
    directly, in degrees.
    """
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(math.degrees(rpy[0]),
                                     math.degrees(rpy[1]),
                                     math.degrees(rpy[2])))
    if scale is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(*scale))


def preview_material(stage, path, rgb, roughness=0.7):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def textured_material(stage, path, texture_rel):
    """UsdPreviewSurface fed by the course texture.

    The lane markings are IN this PNG. Nothing in the geometry knows where a
    lane is, which is why extract_centreline.py has to recover the centreline
    from the image and why the RL observation treats paint analytically.
    """
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    st_reader = UsdShade.Shader.Define(stage, path + "/stReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")

    tex = UsdShade.Shader.Define(stage, path + "/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_rel)
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        st_reader.ConnectableAPI(), "result")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def build_ground(stage, root, texture_rel):
    """Textured quad for looks, separate thin box for physics.

    Deliberately two prims. PhysX wants a simple convex collider, and a
    textured visual mesh is not that. Keeping them apart also means the
    collider's top face can be pinned at exactly z = 0, which is the height
    everything else in the course is measured from.
    """
    hx, hy = ARENA_X / 2.0, ARENA_Y / 2.0
    mesh = UsdGeom.Mesh.Define(stage, root + "/GroundVisual")
    mesh.CreatePointsAttr([Gf.Vec3f(-hx, -hy, 0.0), Gf.Vec3f(hx, -hy, 0.0),
                           Gf.Vec3f(hx, hy, 0.0), Gf.Vec3f(-hx, hy, 0.0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, 0.0), Gf.Vec3f(hx, hy, 0.0)])
    # UVs match meshes/ground.dae: (0,0) at the -x,-y corner, (1,1) at +x,+y.
    primvars = UsdGeom.PrimvarsAPI(mesh)
    st = primvars.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                UsdGeom.Tokens.varying)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    set_pose(mesh.GetPrim(), (MODEL_OFFSET[0], MODEL_OFFSET[1], 0.0))

    mat = textured_material(stage, root + "/Looks/Ground", texture_rel)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mat)

    slab = UsdGeom.Cube.Define(stage, root + "/GroundCollider")
    slab.CreateSizeAttr(1.0)
    set_pose(slab.GetPrim(),
             (MODEL_OFFSET[0], MODEL_OFFSET[1], -GROUND_THICKNESS / 2.0),
             scale=(ARENA_X, ARENA_Y, GROUND_THICKNESS))
    UsdPhysics.CollisionAPI.Apply(slab.GetPrim())
    UsdGeom.Imageable(slab).MakeInvisible()
    return mesh, slab


def build_course(stage, root, world, texture_rel):
    counts = {}
    build_ground(stage, root, texture_rel)

    orange = preview_material(stage, root + "/Looks/Barrel", (0.85, 0.35, 0.05))
    grey = preview_material(stage, root + "/Looks/Barricade", (0.18, 0.18, 0.18))
    green = preview_material(stage, root + "/Looks/Ramp", (0.15, 0.5, 0.18))

    for i, item in enumerate(world["barrels"]):
        x, y, _, _, _, yaw = item["pose"]
        prim = UsdGeom.Cylinder.Define(stage, "%s/Barrels/barrel_%02d" % (root, i))
        prim.CreateRadiusAttr(BARREL_RADIUS)
        prim.CreateHeightAttr(BARREL_HEIGHT)
        prim.CreateAxisAttr("Z")
        prim.CreateExtentAttr([
            Gf.Vec3f(-BARREL_RADIUS, -BARREL_RADIUS, -BARREL_HEIGHT / 2.0),
            Gf.Vec3f(BARREL_RADIUS, BARREL_RADIUS, BARREL_HEIGHT / 2.0)])
        # model.sdf puts the barrel's ORIGIN on the ground; a USD cylinder is
        # centred, so it has to be lifted by half its height.
        set_pose(prim.GetPrim(), (x, y, BARREL_HEIGHT / 2.0), (0, 0, yaw))
        UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
        UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(orange)
    counts["barrels"] = len(world["barrels"])

    for group, mat, key in (("Barricades", grey, "barricades"),
                            ("Ramps", green, "ramps")):
        for i, item in enumerate(world[key]):
            x, y, z, roll, pitch, yaw = item["pose"]
            sx, sy, sz = item["size"]
            prim = UsdGeom.Cube.Define(stage, "%s/%s/%s_%02d" % (root, group, key[:-1], i))
            prim.CreateSizeAttr(1.0)
            set_pose(prim.GetPrim(), (x, y, z), (roll, pitch, yaw), (sx, sy, sz))
            UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
            UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
            UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)
        counts[key] = len(world[key])
    return counts


# ----------------------------------------------------------------- obstacles --
def write_obstacles(path, world):
    """The colliders as plain numbers, for scan_sim.py.

    Separate from course_meta.yaml on purpose: the centreline is recovered
    from the ground texture, these come from model.sdf. Different sources,
    different scripts, so neither can be silently regenerated without the
    other noticing.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("# GENERATED by research/isaac/scripts/build_course_usd.py\n")
        fh.write("# Source: worlds/models/orange_igvc/model.sdf, with igvc.sdf's\n")
        fh.write("# include offset (0, 16.3) already applied. Frame matches\n")
        fh.write("# course_meta.yaml, the Gazebo sim and my_map_save.pgm.\n")
        fh.write("name: igvc\n")
        fh.write("# x, y, radius -- barrels, as worlds/sim_world_ic.sdf already\n")
        fh.write("# approximates them (cylinder r 0.25, 1.0 m tall)\n")
        fh.write("barrels:\n")
        for b in world["barrels"]:
            fh.write("  - [%.4f, %.4f, %.3f]\n" % (b["pose"][0], b["pose"][1],
                                                   BARREL_RADIUS))
        fh.write("# x, y, yaw, size_x, size_y, size_z\n")
        fh.write("barricades:\n")
        for b in world["barricades"]:
            x, y, z, _, _, yaw = b["pose"]
            fh.write("  - [%.4f, %.4f, %.4f, %.3f, %.3f, %.3f]\n"
                     % (x, y, yaw, b["size"][0], b["size"][1], b["size"][2]))
        fh.write("# x, y, z, roll, pitch, yaw, size_x, size_y, size_z\n")
        fh.write("ramps:\n")
        for r in world["ramps"]:
            x, y, z, roll, pitch, yaw = r["pose"]
            fh.write("  - [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.3f, %.3f, %.3f]\n"
                     % (x, y, z, roll, pitch, yaw,
                        r["size"][0], r["size"][1], r["size"][2]))


def check_barrels_against_course(world, centreline_yaml):
    """How far each barrel sits from the drivable centreline.

    Worth automating because eyeballing the overlay gets this wrong: the
    barrels near the top edge look like a scattering of stray obstacles, and
    measuring says 22 of 34 are inside the corridor with a 1.39 m median. The
    few genuine outliers all sit at y ~ 33, across no man's land -- where the
    course is deliberately unmarked, so there is no corridor for them to be
    inside of and the distance is measured to a line this pipeline routed
    rather than to any paint.
    """
    if not os.path.exists(centreline_yaml):
        return None
    import numpy as np
    import yaml

    with open(centreline_yaml) as fh:
        meta = yaml.safe_load(fh)
    rows = np.array(meta["centreline"], dtype=float)
    a, widths = rows[:, :2], rows[:, 2]
    e = np.roll(a, -1, axis=0) - a
    L2 = np.maximum((e * e).sum(1), 1e-12)

    inside = edge = outside = 0
    dists = []
    for x, y, _ in [(b["pose"][0], b["pose"][1], 0) for b in world["barrels"]]:
        u = np.clip(((x - a[:, 0]) * e[:, 0] + (y - a[:, 1]) * e[:, 1]) / L2, 0, 1)
        d = np.hypot(a[:, 0] + u * e[:, 0] - x, a[:, 1] + u * e[:, 1] - y)
        i = int(d.argmin())
        dists.append(d[i])
        if d[i] <= widths[i] / 2.0:
            inside += 1
        elif d[i] <= widths[i] / 2.0 + 1.0:
            edge += 1
        else:
            outside += 1
    dists = np.array(dists)
    return {"n": len(dists), "median": float(np.median(dists)),
            "max": float(dists.max()), "inside": inside, "edge": edge,
            "outside": outside}


# ------------------------------------------------------------------- overlay --
def write_overlay(path, world, centreline_yaml=None):
    """Draw the parsed obstacles onto ground.png and check they land sensibly.

    This, not the 3D render, is the real verification. A straight-down camera
    has a degenerate up vector, so an overhead view can come out rotated and
    tell you nothing about whether the poses are right. Projecting into the
    texture uses the same world/pixel mapping extract_centreline.py already
    validated against the barricades (worst error 0.46 m), so if the barrels
    land beside the painted lanes and the ramp lands between the two
    top-centre lines, the parse is correct.
    """
    import cv2
    import numpy as np

    img = cv2.imread(GROUND_PNG, cv2.IMREAD_COLOR)
    if img is None:
        return None
    h, w = img.shape[:2]

    def w2p(x, y):
        mx, my = x - MODEL_OFFSET[0], y - MODEL_OFFSET[1]
        return (int(round((mx + ARENA_X / 2.0) / ARENA_X * w)),
                int(round((ARENA_Y / 2.0 - my) / ARENA_Y * h)))

    px_per_m = 0.5 * (w / ARENA_X + h / ARENA_Y)

    if centreline_yaml and os.path.exists(centreline_yaml):
        import yaml
        with open(centreline_yaml) as fh:
            meta = yaml.safe_load(fh)
        pts = np.array([w2p(r[0], r[1]) for r in meta["centreline"]], dtype=np.int32)
        cv2.polylines(img, [pts], True, (0, 0, 255), 2)

    for b in world["barrels"]:
        cv2.circle(img, w2p(b["pose"][0], b["pose"][1]),
                   max(3, int(BARREL_RADIUS * px_per_m)), (0, 140, 255), -1)
    for b in world["barricades"]:
        x, y, _, _, _, yaw = b["pose"]
        sx, sy, _ = b["size"]
        box = cv2.boxPoints((w2p(x, y), (sx * px_per_m, sy * px_per_m),
                             -math.degrees(yaw)))
        cv2.drawContours(img, [np.intp(box)], 0, (255, 0, 255), 3)
    for r in world["ramps"]:
        x, y, _, _, _, yaw = r["pose"]
        sx, sy, _ = r["size"]
        box = cv2.boxPoints((w2p(x, y), (sx * px_per_m, sy * px_per_m),
                             -math.degrees(yaw)))
        cv2.drawContours(img, [np.intp(box)], 0, (0, 200, 0), 3)

    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, img)
    return path


# ------------------------------------------------------------------ snapshot --
def snapshot(usd_path, out_dir):
    """Render the course. Look at this before trusting the numbers."""
    import cv2
    import numpy as np
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import Camera, CameraCfg

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 60.0))
    cfg = sim_utils.UsdFileCfg(usd_path=usd_path)
    cfg.func("/World/Course", cfg)
    sim_utils.DomeLightCfg(intensity=1200.0).func(
        "/World/light", sim_utils.DomeLightCfg(intensity=1200.0))

    cam = Camera(CameraCfg(prim_path="/World/cam", update_period=0.0,
                           height=720, width=1280, data_types=["rgb"],
                           spawn=sim_utils.PinholeCameraCfg(
                               focal_length=18.0, clipping_range=(0.1, 500.0))))
    sim.reset()

    views = [("overhead", (0.0, 16.3, 46.0), (0.0, 16.3, 0.0)),
             ("oblique", (26.0, -12.0, 20.0), (0.0, 16.0, 0.0)),
             ("ramp", (6.0, 25.0, 4.0), (0.0, 29.3, 0.3))]
    frames = []
    os.makedirs(out_dir, exist_ok=True)
    for name, eye, tgt in views:
        cam.set_world_poses_from_view(
            torch.tensor([list(eye)], device=cam.device),
            torch.tensor([list(tgt)], device=cam.device))
        for _ in range(6):
            sim.step()
            sim.render()
            cam.update(1.0 / 60.0)
        rgb = cam.data.output["rgb"][0].detach().cpu().numpy()
        if rgb.dtype != np.uint8:
            rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
        bgr = cv2.cvtColor(rgb[..., :3], cv2.COLOR_RGB2BGR)
        cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(bgr, name, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 255), 2)
        path = os.path.join(out_dir, "course_%s.png" % name)
        cv2.imwrite(path, bgr)
        frames.append(bgr)
        print("  rendered %s -> %s" % (name, path))

    sheet = np.vstack([np.hstack(frames[:2]),
                       np.pad(frames[2], ((0, 0), (0, frames[0].shape[1]), (0, 0)))])
    sheet_path = os.path.join(out_dir, "course_contact_sheet.png")
    cv2.imwrite(sheet_path, sheet)
    print("  contact sheet: %s" % sheet_path)


def main():
    if not os.path.exists(args.sdf):
        sys.exit("missing %s" % args.sdf)
    world = parse_world(args.sdf)
    os.makedirs(args.out_dir, exist_ok=True)

    tex_dir = os.path.join(args.out_dir, "textures")
    os.makedirs(tex_dir, exist_ok=True)
    shutil.copy2(GROUND_PNG, os.path.join(tex_dir, "ground.png"))

    usd_path = os.path.join(args.out_dir, args.name)
    stage = Usd.Stage.CreateNew(usd_path) if not os.path.exists(usd_path) \
        else Usd.Stage.Open(usd_path)
    stage.GetRootLayer().Clear()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Course")
    stage.SetDefaultPrim(root.GetPrim())

    counts = build_course(stage, "/Course", world, "./textures/ground.png")
    stage.GetRootLayer().Save()

    write_obstacles(args.obstacles, world)

    print("wrote %s" % usd_path)
    for key in ("barrels", "barricades", "ramps"):
        print("  %-11s %d" % (key, counts.get(key, 0)))
    xs = [b["pose"][0] for b in world["barrels"]]
    ys = [b["pose"][1] for b in world["barrels"]]
    if xs:
        print("  barrel extent: x %.2f..%.2f  y %.2f..%.2f"
              % (min(xs), max(xs), min(ys), max(ys)))
    print("  arena spans x %.1f..%.1f  y %.1f..%.1f (igvc.sdf frame)"
          % (MODEL_OFFSET[0] - ARENA_X / 2, MODEL_OFFSET[0] + ARENA_X / 2,
             MODEL_OFFSET[1] - ARENA_Y / 2, MODEL_OFFSET[1] + ARENA_Y / 2))
    print("  obstacles -> %s" % args.obstacles)

    stats = check_barrels_against_course(
        world, os.path.join(ISAAC_DIR, "config", "course_meta.yaml"))
    if stats:
        print("  barrels vs the drivable corridor: %d inside, %d within 1 m of "
              "the edge, %d beyond" % (stats["inside"], stats["edge"], stats["outside"]))
        print("    median %.2f m from the centreline, max %.2f m"
              % (stats["median"], stats["max"]))
        if stats["outside"] > 8:
            print("    WARNING: that is a lot of barrels off the course. Check the"
                  " include offset in MODEL_OFFSET.")

    overlay = write_overlay(
        os.path.join(args.out_dir, "snapshots", "course_overlay.png"), world,
        os.path.join(ISAAC_DIR, "config", "course_meta.yaml"))
    if overlay:
        print("  overlay   -> %s  -- CHECK THIS: barrels should sit beside the"
              " painted lanes," % overlay)
        print("               the ramp between the two top-centre lines, and the"
              " four barricades")
        print("               on the four black strokes.")

    if args.snapshot:
        print("\nrendering...")
        snapshot(usd_path, os.path.join(args.out_dir, "snapshots"))
    return 0


if __name__ == "__main__":
    try:
        code = main()
    finally:
        simulation_app.close()
    sys.exit(code)
