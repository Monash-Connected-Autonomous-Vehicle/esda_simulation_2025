#!/usr/bin/env python3
"""Drive the vehicle around the Isaac course by hand, and watch what it senses.

WHY THIS EXISTS
    Before trusting a trained policy you want to know two things that no
    reward curve answers: does the vehicle FEEL right, and is the observation
    it receives actually sane? Driving it yourself answers both in about a
    minute. It is also the fastest way to notice sim problems -- a robot that
    understeers, drifts, catches on the ramp or sees phantom obstacles is
    obvious at the wheel and invisible in a training log.

KEYS -- deliberately the same as scripts/teleop_wasd.py
    So the same fingers drive the robot in Gazebo, in Isaac and on the real
    vehicle.

        w / s   forward / reverse
        a / d   turn left / right
        x       stop
        u / j   raise / lower max speed by 10%
        i / k   raise / lower max turn rate by 10%
        r       reset to the course start
        p       print a full observation summary
        q       quit

    Commands go through obs_core.action_to_twist, the same rate-limited path
    the policy uses, so what you feel is what it will feel.

THE READOUT
    One line per control step: commanded and actual speed, lateral offset from
    the lane centreline with the corridor half-width for comparison, the
    nearest obstacle, and the closest synthetic scan return. If the scan
    disagrees with what you can see on screen, the observation is wrong and no
    amount of training will fix it.

USAGE
    C:\\Users\\Ishaan\\IsaacLab\\isaaclab.bat -p research\\isaac\\scripts\\teleop.py --viz kit
    ... --course flat      plain ground, no course geometry
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
parser.add_argument("--course", choices=("igvc", "flat"), default="igvc",
                    help="'igvc' loads the built course USD; 'flat' is bare ground")
parser.add_argument("--hud-hz", type=float, default=4.0,
                    help="how often to print the readout")
parser.add_argument("--max-steps", type=int, default=0,
                    help="stop after this many control steps; 0 runs until you "
                         "press q. Used to smoke-test without a window.")
parser.add_argument("--demo", action="store_true",
                    help="drive forward automatically instead of reading keys, "
                         "so the readout can be checked without keyboard focus")

from isaaclab.app import AppLauncher  # noqa: E402

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402

import course_bank as CB  # noqa: E402
import obs_core as OC  # noqa: E402
import scan_sim_torch as ST  # noqa: E402
from scan_sim import ScanCfg  # noqa: E402

from esda_isaac.assets.esda_robot import (  # noqa: E402
    ESDA_ROBOT_CFG,
    SPAWN_HEIGHT,
    WHEEL_COMMAND_SIGN,
    apply_friction_materials,
)

PHYSICS_DT = 1.0 / 200.0
DECIMATION = 25
CONTROL_DT = PHYSICS_DT * DECIMATION

COURSE_USD = os.path.join(ISAAC_DIR, "usd", "course", "igvc_course.usda")
# scripts/ui_launch.py spawns the robot here in every igvc world.
SPAWN_XY = (11.0, 0.0)


class Keys:
    """Held-key state, so steering is continuous rather than per-keystroke."""

    MOVE = {"W": (1, 0), "S": (-1, 0), "A": (0, 1), "D": (0, -1)}
    SCALE = {"U": (1.1, 1.0), "J": (1 / 1.1, 1.0),
             "I": (1.0, 1.1), "K": (1.0, 1 / 1.1)}

    def __init__(self):
        self.held = set()
        self.speed = OC.V_MAX
        self.turn = OC.W_MAX
        self.quit = False
        self.reset = False
        self.report = False
        self._iface = carb.input.acquire_input_interface()
        self._kb = omni.appwindow.get_default_app_window().get_keyboard()
        self._sub = self._iface.subscribe_to_keyboard_events(self._kb, self._on_key)

    def _on_key(self, event, *_):
        name = event.input.name
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if name in self.MOVE:
                self.held.add(name)
            elif name in self.SCALE:
                ds, dt = self.SCALE[name]
                self.speed = float(np.clip(self.speed * ds, 0.05, OC.V_MAX))
                self.turn = float(np.clip(self.turn * dt, 0.05, OC.W_MAX))
                print("  [limits] speed %.2f m/s  turn %.2f rad/s"
                      % (self.speed, self.turn))
            elif name == "X":
                self.held.clear()
            elif name == "Q":
                self.quit = True
            elif name == "R":
                self.reset = True
            elif name == "P":
                self.report = True
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.held.discard(name)
        return True

    def command(self):
        """Held keys -> a (linear.x, angular.z) request."""
        vx = tw = 0.0
        for key in self.held:
            dx, dth = self.MOVE[key]
            vx += dx
            tw += dth
        return (float(np.clip(vx, -1, 1)) * self.speed,
                float(np.clip(tw, -1, 1)) * self.turn)


def yaw_of(q):
    x, y, z, w = (float(v) for v in q)      # XYZW
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=PHYSICS_DT))
    sim.set_camera_view([SPAWN_XY[0] + 6, SPAWN_XY[1] - 8, 5.0],
                        [SPAWN_XY[0], SPAWN_XY[1], 0.0])

    ground = sim_utils.GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0, dynamic_friction=1.0, restitution=0.0))
    ground.func("/World/ground", ground)
    sim_utils.DomeLightCfg(intensity=1800.0).func(
        "/World/light", sim_utils.DomeLightCfg(intensity=1800.0))

    if args.course == "igvc":
        if not os.path.exists(COURSE_USD):
            sys.exit("missing %s -- run build_course_usd.py first" % COURSE_USD)
        cfg = sim_utils.UsdFileCfg(usd_path=COURSE_USD)
        cfg.func("/World/Course", cfg)

    robot_cfg = ESDA_ROBOT_CFG.replace(prim_path="/World/Robot")
    robot_cfg.init_state = robot_cfg.init_state.replace(
        pos=(SPAWN_XY[0], SPAWN_XY[1], SPAWN_HEIGHT))
    robot = Articulation(robot_cfg)
    apply_friction_materials("/World/Robot")
    sim.reset()

    names = robot.data.joint_names
    jl, jr = names.index("left_wheel_joint"), names.index("right_wheel_joint")

    course = CB.igvc_course()
    centre = torch.as_tensor(course["centre"], dtype=torch.float32, device=sim.device)
    widths = torch.as_tensor(course["widths"], dtype=torch.float32, device=sim.device)
    discs = torch.as_tensor(course["discs"], dtype=torch.float32, device=sim.device)
    radii = torch.as_tensor(course["radii"], dtype=torch.float32, device=sim.device)
    left = torch.as_tensor(course["left"], dtype=torch.float32, device=sim.device)
    right = torch.as_tensor(course["right"], dtype=torch.float32, device=sim.device)
    lane_p0 = torch.cat([left[:-1], right[:-1]], dim=0)
    lane_p1 = torch.cat([left[1:], right[1:]], dim=0)
    scan_cfg = ScanCfg()

    keys = Keys() if not args.demo else None
    print(__doc__.split("KEYS")[1].split("USAGE")[0].rstrip())
    print("  driving. Click the viewport so it has keyboard focus.\n")

    twist = np.zeros(2)
    target = torch.zeros_like(robot.data.joint_vel)
    every = max(1, int(round(1.0 / max(args.hud_hz, 0.1) / CONTROL_DT)))
    step = 0

    while simulation_app.is_running() and not (keys and keys.quit):
        if args.max_steps and step >= args.max_steps:
            break
        want = keys.command() if keys else (0.6 * OC.V_MAX, 0.0)
        action = OC.twist_to_action(want)
        twist = OC.action_to_twist(action, twist, dt=CONTROL_DT)
        wl, wr = OC.twist_to_wheels(twist[0], twist[1])
        target[:, jl] = WHEEL_COMMAND_SIGN * float(wl)
        target[:, jr] = WHEEL_COMMAND_SIGN * float(wr)

        for _ in range(DECIMATION):
            robot.set_joint_velocity_target(target)
            robot.write_data_to_sim()
            sim.step()
            robot.update(PHYSICS_DT)

        if keys and keys.reset:
            keys.reset = False
            root = robot.data.default_root_state.clone()
            root[:, 0], root[:, 1], root[:, 2] = SPAWN_XY[0], SPAWN_XY[1], SPAWN_HEIGHT
            robot.write_root_state_to_sim(root)
            zero = torch.zeros_like(robot.data.joint_pos)
            robot.write_joint_state_to_sim(zero, zero.clone())
            twist[:] = 0.0
            print("  [reset] back to the course start")

        step += 1
        if step % every and not (keys and keys.report):
            continue

        xy = robot.data.root_pos_w[0, :2]
        yaw = yaw_of(robot.data.root_quat_w[0])
        d = torch.linalg.norm(centre - xy[None, :], dim=1)
        i = int(torch.argmin(d))
        nxt = (i + 1) % centre.shape[0]
        tang = centre[nxt] - centre[i]
        tang = tang / torch.clamp(torch.linalg.norm(tang), min=1e-6)
        rel = xy - centre[i]
        lateral = float(rel[0] * -tang[1] + rel[1] * tang[0])
        half = float(widths[i]) / 2.0

        pose = torch.tensor([[float(xy[0]), float(xy[1]), yaw]], device=sim.device)
        ranges = ST.synth_scan_batch(pose, scan_cfg, discs=discs, disc_radii=radii,
                                     lane_p0=lane_p0, lane_p1=lane_p1,
                                     device=sim.device)
        pooled = ST.pool_scan_batch(ranges)[0]
        nearest_scan = float(pooled.min())
        gap = torch.linalg.norm(discs - xy[None, :], dim=1) - radii
        gap = torch.where(radii > 0, gap, torch.full_like(gap, 1e3))
        nearest_obs = float(gap.min())

        v_actual = float(robot.data.root_lin_vel_b[0, 0])
        w_actual = float(robot.data.root_ang_vel_w[0, 2])
        flag = "  <-- OUTSIDE THE LANE" if abs(lateral) > half else ""
        print("  v %+5.2f/%+5.2f m/s  w %+5.2f/%+5.2f rad/s | lateral %+5.2f "
              "(half %.2f) | obstacle %5.2f  scan %5.2f%s"
              % (v_actual, twist[0], w_actual, twist[1], lateral, half,
                 nearest_obs, nearest_scan, flag))

        if keys and keys.report:
            keys.report = False
            print("\n  --- observation summary ---")
            print("  pose           x %.2f  y %.2f  yaw %.3f rad" % (xy[0], xy[1], yaw))
            print("  centreline     sample %d of %d, corridor %.2f m wide"
                  % (i, centre.shape[0], 2 * half))
            print("  pooled scan    %d beams, min %.2f  median %.2f  max %.2f m"
                  % (pooled.numel(), nearest_scan, float(pooled.median()),
                     float(pooled.max())))
            print("  beams under the robot radius (%.2f m): %d"
                  % (OC.ROBOT_RADIUS, int((pooled < OC.ROBOT_RADIUS).sum())))
            print("  nearest obstacle %.2f m (collision terminates under %.2f)"
                  % (nearest_obs, OC.ROBOT_RADIUS))
            print("  ---------------------------\n")

    print("\n  stopped.")
    return 0


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
    finally:
        simulation_app.close()
    sys.exit(code)
