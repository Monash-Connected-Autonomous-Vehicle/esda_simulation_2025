#!/usr/bin/env python3
"""Phase 0 gate: does Isaac Lab actually run against the installed Isaac Sim?

WHY A PURPOSE-BUILT SCRIPT
    The obvious gate is scripts/tutorials/00_sim/create_empty.py, and it is the
    wrong one: it ends in `while simulation_app.is_running(): sim.step()` and
    never exits.  Run headless it spins forever, which from the outside is
    indistinguishable from a hang -- and leads you to suspect the Isaac Lab 3.0
    / Isaac Sim 6.0 pairing when nothing is wrong at all.

    This starts the app, builds a ground plane and a light, steps physics a
    fixed number of times, prints what it saw, and closes.  It terminates, and
    its exit code means something.

USAGE
    C:\\Users\\Ishaan\\IsaacLab\\isaaclab.bat -p research\\isaac\\scripts\\gate_check.py
    ... --steps 200
"""
import argparse
import sys

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--steps", type=int, default=120,
                    help="physics steps to run before closing")

from isaaclab.app import AppLauncher  # noqa: E402

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
# Headless is the default in Isaac Lab 3.0; --viz opts in. Passing --headless
# is deprecated and prints a warning.
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402


def main():
    import isaacsim.core.version as version

    sim_cfg = sim_utils.SimulationCfg(dt=1.0 / 200.0)
    sim = sim_utils.SimulationContext(sim_cfg)

    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=2000.0).func(
        "/World/light", sim_utils.DomeLightCfg(intensity=2000.0))

    sim.reset()
    for _ in range(args.steps):
        sim.step()

    try:
        isaac_version = version.get_version()[0]
    except Exception:
        isaac_version = "unknown"

    print("")
    print("=" * 60)
    print("PHASE 0 GATE: PASS")
    print("  Isaac Sim      : %s" % isaac_version)
    print("  physics dt     : %g s  (%d steps run)" % (sim_cfg.dt, args.steps))
    print("  device         : %s" % sim.device)
    print("  backend        : PhysX (the default; Newton needs NewtonManagerCfg)")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    try:
        code = main()
    finally:
        simulation_app.close()
    sys.exit(code)
