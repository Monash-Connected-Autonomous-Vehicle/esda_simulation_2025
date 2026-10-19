#!/usr/bin/env python3
"""fix_capture_poses.py -- repair a capture whose poses were written with the
wrong spawn offset, using the raw odometry each frame already stored.

WHY THIS EXISTS.  capture_labelled.py writes both the composed world pose
("x", "y", "yaw") and the raw wheel odometry it came from ("odom").  Job 109
ran without --spawn, which used to default to 0,0,0, so all 672 frames of
igvc_oval_slow_test carry world poses that are off by the real spawn
(11.86, -2.31, 1.263 rad).  The frames themselves are fine and the odometry is
fine; only the composition was wrong.  Recomposing costs seconds, where
recapturing costs ten minutes of wall clock and a fresh roll of the dice on
whether the robot wanders somewhere useful.

    python3 vision/fix_capture_poses.py --capture ~/esda_capture/eval_... --check
    python3 vision/fix_capture_poses.py --capture ~/esda_capture/eval_...

--check reports and writes nothing.  Without it, jsons whose stored pose
disagrees with odom+spawn are rewritten in place and stamped
"pose_fixed_from": "odom", so a second run is a no-op rather than a second
rotation.  A frame that has no "odom" key cannot be repaired and is counted,
not guessed at.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_spawns


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--capture", required=True)
    p.add_argument("--world", default="", help="override the world in the json")
    p.add_argument("--spawn", default="", help="x,y,yaw; default from the table")
    p.add_argument("--repo", default=os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))
    p.add_argument("--check", action="store_true", help="report, write nothing")
    p.add_argument("--tol", type=float, default=1e-6)
    a = p.parse_args()

    js = sorted(f for f in os.listdir(a.capture) if f.endswith(".json"))
    if not js:
        raise SystemExit(f"no frame jsons in {a.capture}")

    first = json.load(open(os.path.join(a.capture, js[0])))
    world = a.world or first.get("world", "")
    if a.spawn:
        spawn = tuple(float(v) for v in (a.spawn.split(",") + ["0", "0", "0"])[:3])
    else:
        spawn = world_spawns.spawn_of(world, a.repo)
        if spawn is None:
            raise SystemExit(f"'{world}' is not in {world_spawns.REL}; "
                             f"pass --spawn x,y,yaw")
    print(f"capture {a.capture}")
    print(f"world   {world}   spawn {spawn}")

    fixed = already = ok = no_odom = 0
    worst = 0.0
    for name in js:
        path = os.path.join(a.capture, name)
        d = json.load(open(path))
        od = d.get("odom")
        if not od or len(od) < 3:
            no_odom += 1
            continue
        x, y, yaw = world_spawns.odom_to_world(od[0], od[1], od[2], spawn)
        off = max(abs(x - d.get("x", 0.0)), abs(y - d.get("y", 0.0)))
        worst = max(worst, off)
        if off <= a.tol:
            (already if d.get("pose_fixed_from") else ok)
            if d.get("pose_fixed_from"):
                already += 1
            else:
                ok += 1
            continue
        if not a.check:
            d["pose_was"] = [d.get("x"), d.get("y"), d.get("yaw")]
            d["x"], d["y"], d["yaw"] = x, y, yaw
            d["pose_fixed_from"] = "odom"
            d["spawn_used"] = list(spawn)
            with open(path, "w") as fh:
                json.dump(d, fh)
        fixed += 1

    verb = "would rewrite" if a.check else "rewrote"
    print(f"{len(js)} frames: {verb} {fixed}, already correct {ok}, "
          f"already fixed {already}, no odom {no_odom}")
    print(f"largest disagreement between stored pose and odom+spawn: "
          f"{worst:.3f} m")
    if no_odom:
        print(f"WARNING {no_odom} frames carry no raw odom and cannot be "
              f"repaired -- they were written by an older capture_labelled.py")
    if a.check and fixed:
        print("run again without --check to rewrite them")
    # A capture that needs no fixing and a capture that cannot be fixed look
    # the same to a job script unless it reads this exit code.
    return 2 if (no_odom and no_odom == len(js)) else 0


if __name__ == "__main__":
    raise SystemExit(main())
