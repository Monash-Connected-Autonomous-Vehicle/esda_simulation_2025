#!/usr/bin/env python3
"""Convert the exported URDF into a USD articulation Isaac Lab can spawn.

INPUT comes from tools/export_urdf_for_isaac.py, which has already stripped the
<gazebo> extensions and replaced the xacro's toy masses -- see
research/isaac/config/robot_mass.yaml for why a 1.25 kg robot would have been
unusable under PhysX.

THREE CHOICES THAT MATTER

  fix_base=False
      It is a mobile robot. A fixed base bolts it to the origin and every
      velocity command does nothing, which looks like a broken controller.

  target_type="velocity"
      The policy commands wheel velocities, exactly as diff_drive_controller
      does on the real robot. A position drive imposes a PD loop that fights
      the policy's output. The actual gains are owned by the ArticulationCfg
      in esda_isaac/assets/esda_robot.py -- these are defaults baked into the
      USD and are overridden at spawn time.

  merge_fixed_joints=True
      base_link carries no <inertial> of its own, so on its own it would be a
      zero-mass articulation root. Merging folds chassis, caster, lidar pole,
      laser_frame, camera and gps into it, which is both correct and what
      gives the root its 48 kg. The two wheel joints are `continuous` and
      survive as real joints.

WHAT THE IMPORTER CANNOT CARRY OVER
  Friction. robot_core_ref.xacro expresses the caster's mu 0.001 inside a
  <gazebo> block, which the exporter strips and the importer would ignore
  anyway. Without it the caster is a high-friction sphere welded to the
  chassis -- a brake, not a skid, and the robot would struggle to turn.
  --materials applies it afterwards from robot_mass.yaml.

  Cylinder wheels. Isaac Sim 6's importer dropped
  replace_cylinders_with_capsules, so wheel collisions become faceted convex
  hulls. Tolerable at <= 1 m/s on flat ground; revisit if traction looks odd.

USAGE
    C:\\Users\\Ishaan\\IsaacLab\\isaaclab.bat -p research\\isaac\\scripts\\convert_robot.py
    ... --no-materials      convert only, leave friction alone
    ... --describe-only     re-inspect an already converted USD
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_URDF = os.path.join(REPO, "research", "isaac", "assets", "esda_robot.urdf")
DEFAULT_USD_DIR = os.path.join(REPO, "research", "isaac", "usd")
DEFAULT_MASS_CFG = os.path.join(REPO, "research", "isaac", "config", "robot_mass.yaml")

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--urdf", default=DEFAULT_URDF)
parser.add_argument("--usd-dir", default=DEFAULT_USD_DIR)
parser.add_argument("--usd-name", default="esda_robot.usd")
parser.add_argument("--mass-config", default=DEFAULT_MASS_CFG)
parser.add_argument("--no-materials", action="store_true",
                    help="skip the friction pass")
parser.add_argument("--describe-only", action="store_true",
                    help="do not convert; just inspect the existing USD")
parser.add_argument("--flat", action="store_true",
                    help="skip the asset transformer and emit a single flat USD "
                         "instead of the layered Geometry/Physics split")
parser.add_argument("--drive-stiffness", type=float, default=1000.0,
                    help="velocity-drive strength baked into the USD; the "
                         "ArticulationCfg overrides this at spawn time")

from isaaclab.app import AppLauncher  # noqa: E402

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402


# ------------------------------------------------------------------ convert --
def clear_stale_outputs(usd_dir, stem):
    """Remove previous conversions of this asset so the output path is stable.

    Isaac Lab's converter refuses to overwrite: given an existing esda_robot/
    it writes esda_robot_1/, then _2, and so on. The ArticulationCfg would
    then either point at a stale asset or have to guess which directory is
    newest -- both worse than deleting output that research/isaac/usd/ is
    gitignored for and that this script regenerates in seconds.

    Scoped deliberately: only directories DIRECTLY under usd_dir whose name is
    the stem or the stem plus a numeric suffix.
    """
    import re
    import shutil

    pattern = re.compile(r"^%s(_\d+)?$" % re.escape(stem))
    removed = []
    if os.path.isdir(usd_dir):
        for name in sorted(os.listdir(usd_dir)):
            path = os.path.join(usd_dir, name)
            if os.path.isdir(path) and pattern.match(name):
                shutil.rmtree(path)
                removed.append(name)
    return removed


def convert(urdf_path, usd_dir, usd_name, drive_stiffness, flat=False):
    cfg = UrdfConverterCfg(
        run_asset_transformer=not flat,
        asset_path=urdf_path,
        usd_dir=usd_dir,
        usd_file_name=usd_name,
        force_usd_conversion=True,
        # Instanceable matters once the training scene clones this thousands of
        # times; non-instanceable imports are a known failure for cloned envs.
        # Instanceable matters once the training scene clones this thousands
        # of times. Tested with False while chasing the upside-down asset: it
        # changed nothing, so instancing is NOT implicated in that bug.
        make_instanceable=True,
        fix_base=False,
        merge_fixed_joints=True,
        self_collision=False,
        collision_type="Convex Hull",
        robot_type="Wheeled",
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            drive_type="force",
            target_type="velocity",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=drive_stiffness,
                damping=0.0,
            ),
        ),
    )
    converter = UrdfConverter(cfg)
    return converter.usd_path


# ---------------------------------------------------------------- materials --
def load_material_cfg(path):
    import yaml
    with open(path) as fh:
        return (yaml.safe_load(fh) or {}).get("materials") or {}


def classify(prim):
    """Which friction material a collider should get.

    Name matching alone does not work.  merge_fixed_joints folds caster_wheel,
    chassis, lidar_pole and laser_frame into base_link and renames their
    colliders generically -- they come out as sphere_1, box_3, box_4,
    cylinder_5, with every trace of "caster" gone.  The wheels keep their names
    only because they hang off real joints and so stay separate bodies.

    So the caster is identified by shape instead: it is the only Sphere
    collider in the whole robot (robot_core_ref.xacro gives it
    caster_wheel_radius 0.08 and nothing else is spherical).  main() asserts
    that there is exactly one, so if the description ever grows a second
    sphere this fails loudly rather than silently binding the wrong friction.
    """
    path = str(prim.GetPath()).lower()
    # Name first. Now that the caster is a real swivel-and-roll assembly it
    # hangs off continuous joints, so merge_fixed_joints leaves it as its own
    # body with its name intact.
    if "caster" in path:
        return "caster"
    if "wheel" in path:
        return "wheel"
    # Shape fallback, for the legacy asset where the caster was a single rigid
    # sphere merged into base_link and renamed to a bare "sphere_N".
    if prim.IsA(UsdGeom.Sphere):
        return "caster"
    return "body"


def apply_materials(usd_path, materials):
    """Bind PhysX friction materials to the collider prims.

    The importer gives every collider the stage default, so without this the
    caster -- which robot_core_ref.xacro deliberately made nearly frictionless
    with mu 0.001 -- grips exactly as hard as the drive wheels. A trailing
    sphere with wheel-grade friction is a brake: the robot resists yaw and
    scrubs instead of turning.
    """
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print("  WARNING: could not open %s for the friction pass" % usd_path)
        return {}

    scope = "/Materials_physics"
    created = {}
    for name, spec in sorted(materials.items()):
        path = "%s/%s" % (scope, name)
        mat = UsdShade.Material.Define(stage, path)
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr().Set(float(spec.get("static_friction", 0.5)))
        api.CreateDynamicFrictionAttr().Set(float(spec.get("dynamic_friction", 0.5)))
        api.CreateRestitutionAttr().Set(float(spec.get("restitution", 0.0)))
        created[name] = mat

    bound = {}
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        kind = classify(prim)
        mat = created.get(kind)
        if mat is None:
            continue
        UsdShade.MaterialBindingAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(
            mat, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics")
        bound.setdefault(kind, []).append(str(prim.GetPath()))

    stage.GetRootLayer().Save()
    return bound


# ----------------------------------------------------------------- describe --
def describe_shape(prim):
    """Human-readable size, so a wrong scale is visible rather than implied."""
    try:
        if prim.IsA(UsdGeom.Sphere):
            return "r=%.4f" % float(UsdGeom.Sphere(prim).GetRadiusAttr().Get() or 0)
        if prim.IsA(UsdGeom.Cylinder):
            cyl = UsdGeom.Cylinder(prim)
            return "r=%.4f h=%.4f" % (float(cyl.GetRadiusAttr().Get() or 0),
                                      float(cyl.GetHeightAttr().Get() or 0))
        if prim.IsA(UsdGeom.Cube):
            return "size=%.4f" % float(UsdGeom.Cube(prim).GetSizeAttr().Get() or 0)
    except Exception:
        pass
    return ""


def describe_drive(prim):
    """The joint drive as PhysX will apply it.

    Worth printing rather than assuming: a position drive here would quietly
    impose a PD loop that fights every velocity the policy commands, and the
    symptom is a sluggish robot rather than an error.
    """
    for token in ("angular", "linear"):
        drive = UsdPhysics.DriveAPI.Get(prim, token)
        if not drive:
            continue

        def val(attr, default=0.0):
            a = attr()
            return float(a.Get()) if a and a.Get() is not None else default

        dtype = drive.GetTypeAttr().Get() if drive.GetTypeAttr() else "?"
        stiff = val(drive.GetStiffnessAttr)
        damp = val(drive.GetDampingAttr)
        force = val(drive.GetMaxForceAttr, float("inf"))
        mode = "VELOCITY" if (stiff > 0.0 and damp == 0.0) else (
            "position" if stiff > 0.0 else "free")
        return "%s drive, stiffness %.1f damping %.1f maxForce %.3g  [%s]" % (
            dtype, stiff, damp, force, mode)
    return "no DriveAPI -- joint is free"


def describe(usd_path):
    """Print the articulation as PhysX will see it. This is the verification."""
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print("  could not open %s" % usd_path)
        return

    # Up axis and metres-per-unit. A Y-up asset dropped into Isaac's Z-up world
    # comes out rotated, which looks exactly like a robot built upside down.
    print("  stage: upAxis = %s, metersPerUnit = %s"
          % (UsdGeom.GetStageUpAxis(stage), UsdGeom.GetStageMetersPerUnit(stage)))
    default_prim = stage.GetDefaultPrim()
    if default_prim:
        xf = UsdGeom.Xformable(default_prim)
        ops = xf.GetOrderedXformOps() if xf else []
        print("  default prim %s, xform ops: %s"
              % (default_prim.GetPath(),
                 ", ".join("%s=%s" % (o.GetOpName(), o.Get()) for o in ops) or "none"))

    bodies, joints, colliders, total_mass = [], [], [], 0.0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            mass = 0.0
            if prim.HasAPI(UsdPhysics.MassAPI):
                attr = UsdPhysics.MassAPI(prim).GetMassAttr()
                mass = float(attr.Get() or 0.0)
            bodies.append((path, mass))
            total_mass += mass
        if prim.IsA(UsdPhysics.Joint):
            jtype = prim.GetTypeName()
            joints.append((path, str(jtype)))
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            colliders.append(path)

    print("\n  rigid bodies (%d):" % len(bodies))
    for path, mass in bodies:
        print("    %-58s %8.3f kg" % (path[-58:], mass))
    print("    %-58s %8.3f kg total" % ("", total_mass))

    print("\n  joints (%d):" % len(joints))
    for path, jtype in joints:
        prim = stage.GetPrimAtPath(path)
        drive = describe_drive(prim)
        print("    %-44s %-22s %s" % (path[-44:], jtype, drive))

    # World-space extents, because the shape attributes alone cannot tell you
    # whether a wheel is standing up or lying flat. The URDF rotates each wheel
    # by rpy -/+pi/2 so its cylinder axis runs along the robot's Y; if that
    # rotation were lost the wheel would be a flat disc sitting 0.1 m above the
    # ground, spinning in the air while the robot rested on its caster -- which
    # looks exactly like "the drive does nothing".
    print("\n  colliders (%d), position of each collider's origin relative to"
          " the robot root:" % len(colliders))
    print("    %-30s %-9s %-7s %8s %8s %8s  %s"
          % ("prim", "type", "class", "x", "y", "z", "shape + scale"))
    for path in colliders:
        prim = stage.GetPrimAtPath(path)
        try:
            m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            t = m.ExtractTranslation()
            # Scale shows up as the row magnitudes; a Cube of size 1.0 is only
            # meaningful once you know what it was scaled by.
            sx = Gf.Vec3d(m[0][0], m[0][1], m[0][2]).GetLength()
            sy = Gf.Vec3d(m[1][0], m[1][1], m[1][2]).GetLength()
            sz = Gf.Vec3d(m[2][0], m[2][1], m[2][2]).GetLength()
            scale = "scale %.3f %.3f %.3f" % (sx, sy, sz)
            x, y, z = t[0], t[1], t[2]
        except Exception as exc:
            x = y = z = float("nan")
            scale = "scale ? (%s)" % exc
        # PhysX contact/rest offsets. A non-zero rest offset holds colliders
        # that far APART at equilibrium, so a robot with one will hover above
        # the ground and its wheels will spin in the air -- and a spawn gap
        # smaller than the offset counts as penetration, which is exactly what
        # a mysterious upward shove at reset looks like.
        offsets = ""
        api = PhysxSchema.PhysxCollisionAPI(prim)
        if prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
            def off(attr):
                a = attr()
                v = a.Get() if a else None
                return "unset" if v is None else "%.4f" % float(v)
            offsets = "contact %s rest %s" % (off(api.GetContactOffsetAttr),
                                              off(api.GetRestOffsetAttr))
        else:
            offsets = "no PhysxCollisionAPI"
        print("    %-30s %-9s %-7s %8.4f %8.4f %8.4f  %s %s  %s"
              % (path[-30:], prim.GetTypeName(), classify(prim), x, y, z,
                 describe_shape(prim), scale, offsets))
    print("    base_link rides the wheel axle, so with the robot at rest the"
          " ground is at z = -0.1625.")
    print("    Nothing should reach below that, and nothing should overlap the"
          " wheels at |y| >= 0.1915.")

    arts = [str(p.GetPath()) for p in stage.Traverse()
            if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    print("\n  articulation roots: %s" % (", ".join(arts) or "NONE -- this will not spawn"))
    return {"bodies": bodies, "joints": joints, "colliders": colliders,
            "total_mass": total_mass, "roots": arts}


def resolve_usd(usd_dir, usd_name):
    """Where the converter actually puts the asset.

    It writes a DIRECTORY named after the asset containing a .usda, not the
    .usd filename it is handed, so the naive join misses it.
    """
    stem = os.path.splitext(usd_name)[0]
    for candidate in (os.path.join(usd_dir, stem, stem + ".usda"),
                      os.path.join(usd_dir, stem, usd_name),
                      os.path.join(usd_dir, usd_name)):
        if os.path.exists(candidate):
            return candidate
    return os.path.join(usd_dir, usd_name)


def main():
    usd_path = resolve_usd(args.usd_dir, args.usd_name)

    if not args.describe_only:
        if not os.path.exists(args.urdf):
            sys.exit("missing %s -- run tools/export_urdf_for_isaac.py first" % args.urdf)
        os.makedirs(args.usd_dir, exist_ok=True)
        stem = os.path.splitext(args.usd_name)[0]
        stale = clear_stale_outputs(args.usd_dir, stem)
        if stale:
            print("removed stale output: %s" % ", ".join(stale))
        print("converting %s" % args.urdf)
        usd_path = convert(args.urdf, args.usd_dir, args.usd_name,
                           args.drive_stiffness, flat=args.flat)
        print("  -> %s" % usd_path)
        expected = os.path.join(args.usd_dir, stem)
        if not os.path.abspath(usd_path).startswith(os.path.abspath(expected)):
            print("  WARNING: output landed outside %s; the ArticulationCfg's "
                  "stable path will be wrong" % expected)

        if not args.no_materials:
            materials = load_material_cfg(args.mass_config)
            print("\nfriction pass (%d materials from robot_mass.yaml):" % len(materials))
            bound = apply_materials(usd_path, materials)
            for kind in sorted(bound):
                spec = materials[kind]
                print("  %-7s mu_s %.3f mu_d %.3f -> %d collider(s)"
                      % (kind, spec.get("static_friction", 0),
                         spec.get("dynamic_friction", 0), len(bound[kind])))
                for p in bound[kind]:
                    print("      %s" % p[-66:])
            n_caster = len(bound.get("caster", []))
            if n_caster != 1:
                print("  WARNING: expected exactly one sphere collider (the "
                      "caster), found %d." % n_caster)
                print("           Either the skid now has wheel-grade friction "
                      "and will resist turning,")
                print("           or a new spherical collider is being treated "
                      "as the caster. Check the")
                print("           collider table below before trusting any "
                      "driving behaviour.")

    print("\ndescribing %s" % usd_path)
    describe(usd_path)
    return 0


if __name__ == "__main__":
    try:
        code = main()
    finally:
        simulation_app.close()
    sys.exit(code)
