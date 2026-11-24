#!/usr/bin/env python3
"""Does the robot go OVER the ramp, or through it?

WHY THIS EXISTS
    The course USD has colliders on everything in it. The RL environment needs
    most of them OFF, because it decides contact from the course tensors and a
    second parallel representation of a barrel would make what you watch
    differ from what was trained. The first version of that stripping took the
    whole subtree -- including the two ramp slabs, which have no tensor
    representation at all, because the course tensors carry no height.

    The result was a phantom ramp: the robot drove through the only piece of
    terrain the real course has, and every lap measured so far skipped the
    climb without anything in the logs saying so. It was spotted by watching a
    video, which is not a gate.

    So this is the gate. It asks three questions, and the first is the one that
    was answered wrong:

      1. After _set_course_collisions, is collision ON for the ramps and OFF
         for the barrels, barricades and the course's duplicate ground slab?
      2. Does the ramp HOLD? Dropped onto the apex, the robot must come to
         rest about a wheel radius above the ramp surface, not above z = 0.
      3. Can the robot CLIMB it? Driven at the foot of the slope at the speed
         the policy is allowed, it must gain the ramp's height and come down
         the far side, without tipping past the termination threshold.

    Question 2 is the one that fails if the colliders are wrong. Question 3 can
    fail on its own terms -- friction, torque, approach angle -- and that is a
    vehicle finding, not a scene-setup finding.

USAGE
    C:\\Users\\Ishaan\\IsaacLab\\isaaclab.bat -p research\\isaac\\scripts\\ramp_check.py
    ... --viz kit      watch it (--viz takes a backend; a bare --viz errors)
    ... --speed 0.8    approach speed, m/s
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_DIR = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, ISAAC_DIR)
sys.path.insert(0, os.path.join(ISAAC_DIR, "esda_isaac"))

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--speed", type=float, default=0.8,
                    help="approach speed, m/s (the policy's cap is 1.0)")
parser.add_argument("--course", default=os.path.join(ISAAC_DIR, "usd", "course",
                                                     "igvc_course.usda"))
parser.add_argument("--settle_s", type=float, default=3.0,
                    help="seconds to let the drop test settle")
parser.add_argument("--drive_s", type=float, default=12.0,
                    help="seconds to drive the climb test")

from isaaclab.app import AppLauncher  # noqa: E402

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402

import obs_core as OC  # noqa: E402
from esda_isaac.assets.esda_robot import (  # noqa: E402
    ESDA_ROBOT_CFG,
    apply_friction_materials,
    WHEEL_COMMAND_SIGN,
)
from esda_isaac.tasks.course.course_env import (  # noqa: E402
    DRIVE_ON,
    _set_course_collisions,
)

PHYSICS_DT = 1.0 / 200.0
DECIMATION = 25
COURSE_ROOT = "/World/Course"

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    # Detail only on failure: it is phrased as a diagnosis, so printing it
    # under a PASS reads as the opposite of the result.
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name,
                           ("   " + detail) if (detail and not ok) else ""))


def world_bound(prim_path):
    """World-space bounding box of a prim and its descendants.

    Read from the stage rather than recomputed from model.sdf, so the figure
    this test compares against is the geometry that was actually spawned.
    """
    import omni.usd
    from pxr import Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not (prim and prim.IsValid()):
        return None
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if box.IsEmpty():
        return None
    lo, hi = box.GetMin(), box.GetMax()
    return ((lo[0], lo[1], lo[2]), (hi[0], hi[1], hi[2]))


def collision_flags(prim_path):
    """Every collider under a prim, and whether collision is enabled."""
    import omni.usd
    from pxr import UsdPhysics

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(prim_path)
    out = {}
    stack = [root] if root and root.IsValid() else []
    while stack:
        prim = stack.pop()
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            attr = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
            out[str(prim.GetPath())] = bool(attr.Get()) if attr else True
        stack.extend(prim.GetChildren())
    return out


def place(robot, x, y, z, yaw):
    """Teleport the robot, at rest, to a world pose."""
    root = robot.data.default_root_state.clone()
    root[:, 0] = x
    root[:, 1] = y
    root[:, 2] = z
    root[:, 3] = 0.0                 # XYZW, not WXYZ
    root[:, 4] = 0.0
    root[:, 5] = math.sin(yaw * 0.5)
    root[:, 6] = math.cos(yaw * 0.5)
    root[:, 7:] = 0.0
    robot.write_root_state_to_sim(root)
    joints = torch.zeros_like(robot.data.joint_pos)
    robot.write_joint_state_to_sim(joints, joints.clone())


def tilt_of(quat):
    """Angle between the body's own up axis and world up, from XYZW."""
    x, y, z, w = (float(v) for v in quat)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, up_z)))


def step(sim, robot, target, n):
    for _ in range(n):
        robot.set_joint_velocity_target(target)
        robot.write_data_to_sim()
        sim.step()
        robot.update(PHYSICS_DT)


def main():
    if not os.path.exists(args.course):
        print("  FAIL: no course USD at %s" % args.course)
        print("  build it first: research/isaac/scripts/build_course_usd.py")
        return 1

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=PHYSICS_DT))

    gp = sim_utils.GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0, dynamic_friction=1.0, restitution=0.0))
    gp.func("/World/ground", gp)
    sim_utils.DomeLightCfg(intensity=2500.0).func(
        "/World/light", sim_utils.DomeLightCfg(intensity=2500.0))

    cfg = sim_utils.UsdFileCfg(usd_path=args.course)
    cfg.func(COURSE_ROOT, cfg)

    # The unit under test: the same call the environment makes.
    kept, stripped = _set_course_collisions(COURSE_ROOT)
    drive_mat = sim_utils.RigidBodyMaterialCfg(
        static_friction=1.0, dynamic_friction=1.0, restitution=0.0)
    drive_mat.func("/World/courseMaterial", drive_mat)
    for path in kept:
        sim_utils.bind_physics_material(path, "/World/courseMaterial")

    print("\n1. WHICH COLLIDERS SURVIVED")
    print("  drive-on subtrees: %s" % ", ".join(DRIVE_ON))
    print("  %d colliders kept, %d stripped" % (len(kept), stripped))

    flags = collision_flags(COURSE_ROOT)
    on = sorted(p for p, v in flags.items() if v)
    off = sorted(p for p, v in flags.items() if not v)
    print("  enabled: %s" % (", ".join(p.rsplit("/", 1)[-1] for p in on) or "none"))

    want_on = [p for p in flags if any("/%s/" % d in p for d in DRIVE_ON)]
    check("every drive-on collider is enabled",
          want_on and all(flags[p] for p in want_on),
          "%d of %d enabled" % (sum(1 for p in want_on if flags[p]), len(want_on)))
    check("ramps are among them",
          any("ramp" in p.lower() for p in on),
          "found: %s" % ", ".join(p.rsplit("/", 1)[-1] for p in on))

    obstacles = [p for p in flags
                 if any(k in p.lower() for k in ("barrel", "barricade"))]
    check("obstacle colliders are all off",
          obstacles and not any(flags[p] for p in obstacles),
          "%d of %d still enabled"
          % (sum(1 for p in obstacles if flags[p]), len(obstacles)))
    ground = [p for p in flags if "groundcollider" in p.lower()]
    check("the course's duplicate ground slab is off",
          ground and not any(flags[p] for p in ground),
          "%d found, %d enabled"
          % (len(ground), sum(1 for p in ground if flags[p])))

    bound = world_bound("%s/Ramps" % COURSE_ROOT)
    if bound is None:
        print("  FAIL: no Ramps subtree in the course USD")
        return 1
    (rx0, ry0, _), (rx1, ry1, rz1) = bound
    apex_x, apex_y = 0.5 * (rx0 + rx1), 0.5 * (ry0 + ry1)
    print("  ramp extent: x %.2f..%.2f  y %.2f..%.2f  top z %.3f m"
          % (rx0, rx1, ry0, ry1, rz1))
    # From the bounding box, so the rise includes the slab's own thickness and
    # the run reaches the box corner rather than the surface. It overstates the
    # true 8.5 degrees in model.sdf by a fraction of a degree; it is reported
    # for scale, and nothing is checked against it.
    print("  bounding box gives %.2f m rise over %.2f m run (~%.1f deg; "
          "model.sdf authors 8.5)"
          % (rz1, 0.5 * (rx1 - rx0),
             math.degrees(math.atan2(rz1, 0.5 * (rx1 - rx0)))))

    robot = Articulation(ESDA_ROBOT_CFG.replace(prim_path="/World/Robot"))
    apply_friction_materials("/World/Robot")
    sim.reset()

    names = list(robot.data.joint_names)
    left, right = names.index("left_wheel_joint"), names.index("right_wheel_joint")
    zero = torch.zeros_like(robot.data.joint_vel)
    settle_steps = int(args.settle_s / PHYSICS_DT)

    print("\n2. DOES THE RAMP HOLD?")
    # Flat ground first, as the control: the same drop with no ramp under it.
    place(robot, apex_x, ry0 - 6.0, OC.ROBOT_RADIUS + 0.4, 0.0)
    step(sim, robot, zero, settle_steps)
    flat_z = float(robot.data.root_pos_w[0, 2])
    print("  dropped beside the ramp: rests at z = %.3f m" % flat_z)

    # Then the apex. The ridge runs along y, so both wheels sit at the same
    # height and the robot is not asked to straddle a side slope.
    place(robot, apex_x, apex_y, rz1 + OC.ROBOT_RADIUS + 0.4, 0.0)
    step(sim, robot, zero, settle_steps)
    apex_z = float(robot.data.root_pos_w[0, 2])
    lift = apex_z - flat_z
    print("  dropped on the apex:     rests at z = %.3f m (%+.3f m)"
          % (apex_z, lift))

    # A solid ramp lifts the robot by close to its height. A phantom one lifts
    # it by nothing, because the robot falls through to the ground plane. The
    # band is wide because the caster trails off the back of a 4 m ramp and
    # pitches the body, which lowers the axle a little.
    check("the ramp lifts the robot off the ground plane",
          lift > 0.6 * rz1,
          "lifted %.3f m against a %.3f m ramp -- the robot fell through"
          % (lift, rz1))
    check("the robot is not resting inside the ramp",
          apex_z > rz1 * 0.5,
          "axle at %.3f m, ramp top at %.3f m" % (apex_z, rz1))

    print("\n3. CAN IT CLIMB?")
    # Start clear of the foot so the approach is at speed, not from rest on
    # the slope -- the policy will always meet the ramp already moving.
    start_x = rx0 - 2.5
    place(robot, start_x, apex_y, OC.ROBOT_RADIUS + 0.05, 0.0)
    step(sim, robot, zero, int(0.5 / PHYSICS_DT))

    twist = np.array([0.0, 0.0])
    want = (min(args.speed, OC.V_MAX), 0.0)
    max_z, max_tilt, track = -9.9, 0.0, []
    for _ in range(int(args.drive_s / (PHYSICS_DT * DECIMATION))):
        twist = OC.action_to_twist(OC.twist_to_action(want), twist)
        wl, wr = OC.twist_to_wheels(float(twist[0]), float(twist[1]))
        target = torch.zeros_like(robot.data.joint_vel)
        target[:, left] = WHEEL_COMMAND_SIGN * wl
        target[:, right] = WHEEL_COMMAND_SIGN * wr
        step(sim, robot, target, DECIMATION)
        x = float(robot.data.root_pos_w[0, 0])
        z = float(robot.data.root_pos_w[0, 2])
        max_z = max(max_z, z)
        max_tilt = max(max_tilt, tilt_of(robot.data.root_quat_w[0]))
        track.append((x, z))

    end_x = track[-1][0] if track else start_x
    print("  drove x %.2f -> %.2f m" % (start_x, end_x))
    print("  peak axle height %.3f m (flat rest %.3f, ramp top %.3f)"
          % (max_z, flat_z, rz1))
    print("  peak tilt %.1f deg (termination fires at %.1f)"
          % (math.degrees(max_tilt), math.degrees(0.6)))
    for x, z in track[::max(1, len(track) // 12)]:
        bar = "#" * int(max(0.0, (z - flat_z)) * 100)
        print("    x %+6.2f  z %.3f  %s" % (x, z, bar))

    check("the climb gained the ramp's height",
          max_z - flat_z > 0.6 * rz1,
          "gained %.3f m of a %.3f m ramp" % (max_z - flat_z, rz1))
    check("it got over and down the far side",
          end_x > rx1,
          "stopped at x = %.2f, ramp ends at %.2f" % (end_x, rx1))
    check("it stayed upright through the climb",
          max_tilt < 0.6,
          "peak tilt %.1f deg exceeds the 34.4 deg termination"
          % math.degrees(max_tilt))

    print("\n" + "-" * 68)
    failed = [n for n, ok, _ in RESULTS if not ok]
    print("%d of %d checks passed" % (len(RESULTS) - len(failed), len(RESULTS)))
    if failed:
        print("FAILED: %s" % "; ".join(failed))
        return 1
    print("The ramp is solid and the vehicle can drive it.")
    return 0


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except Exception:
        # Print the traceback BEFORE closing the app. simulation_app.close() in
        # a finally block swallows the exception and exits 0, which once made a
        # crash read as a clean run.
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
    sys.exit(code)
