#!/usr/bin/env python3
"""Phase 4 gate: does the course environment build, step and reset sanely?

Constructs the environment directly rather than through gym, so a failure here
is the environment's and not the registration's. Drives three fixed policies
and reports what each produces:

    zero     wheels held still. Nothing should move, nothing should terminate
             for any reason except the episode timing out.
    forward  full throttle, no steering. Reward should be positive and
             dominated by progress; terminations should be collisions and lane
             crossings, not tipping.
    random   uniform actions. Everything should fire, and nothing should ever
             produce a NaN.

USAGE
    C:\\Users\\Ishaan\\IsaacLab\\isaaclab.bat -p research\\isaac\\scripts\\env_check.py
    ... --num_envs 64 --steps 120
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_DIR = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, ISAAC_DIR)
sys.path.insert(0, os.path.join(ISAAC_DIR, "esda_isaac"))

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--num_envs", type=int, default=128)
parser.add_argument("--steps", type=int, default=120)

from isaaclab.app import AppLauncher  # noqa: E402

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch  # noqa: E402

import obs_core as OC  # noqa: E402
from esda_isaac.tasks.course.course_env import (  # noqa: E402
    EsdaCourseEnv,
    EsdaCourseEnvCfg,
)

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name,
                           ("   " + detail) if detail and not ok else ""))


def run(env, action_fn, steps, label):
    """Drive a fixed policy and collect statistics."""
    obs, _ = env.reset()
    rew_sum = torch.zeros(env.num_envs, device=env.device)
    deaths = 0
    nan_obs = nan_rew = 0
    moved = torch.zeros(env.num_envs, device=env.device)
    start = env.robot.data.root_pos_w[:, :2].clone()

    for _ in range(steps):
        action = action_fn(env)
        obs, rew, died, timeout, _ = env.step(action)
        policy = obs["policy"] if isinstance(obs, dict) else obs
        nan_obs += int(torch.isnan(policy).any())
        nan_rew += int(torch.isnan(rew).any())
        rew_sum += rew
        deaths += int(died.sum())
        moved = torch.maximum(
            moved, torch.linalg.norm(env.robot.data.root_pos_w[:, :2] - start, dim=1))

    print("  %-8s reward/ep %+8.3f   deaths %4d   max displacement %5.2f m"
          % (label, float(rew_sum.mean()), deaths, float(moved.max())))
    return {"reward": float(rew_sum.mean()), "deaths": deaths,
            "moved": float(moved.max()), "nan_obs": nan_obs, "nan_rew": nan_rew}


def main():
    cfg = EsdaCourseEnvCfg()
    cfg.scene.num_envs = args.num_envs
    env = EsdaCourseEnv(cfg)

    print("\n  courses in bank : %d (%s)"
          % (env.n_courses, ", ".join(sorted(set(env.course_names)))))
    print("  observation dim : %d (expected %d)"
          % (env.cfg.observation_space, OC.OBS_DIM))
    print("  control rate    : %.1f Hz" % (1.0 / (cfg.sim.dt * cfg.decimation)))
    print("")

    zero = run(env, lambda e: torch.zeros(e.num_envs, OC.ACTION_DIM,
                                          device=e.device), args.steps, "zero")
    fwd = run(env, lambda e: torch.tensor([[1.0, 0.0]], device=e.device)
              .repeat(e.num_envs, 1), args.steps, "forward")
    rnd = run(env, lambda e: torch.rand(e.num_envs, OC.ACTION_DIM,
                                        device=e.device) * 2 - 1,
              args.steps, "random")

    print("")
    # An indexing bug that used env indices where course indices belonged
    # stayed invisible while num_envs <= n_courses and then asserted on the GPU
    # at training scale. Make the gate itself refuse to run in the safe regime.
    check("more environments than courses, so course indexing is exercised",
          env.num_envs > env.n_courses,
          "%d envs vs %d courses -- raise --num_envs"
          % (env.num_envs, env.n_courses))
    check("observations never NaN",
          zero["nan_obs"] == fwd["nan_obs"] == rnd["nan_obs"] == 0)
    check("rewards never NaN",
          zero["nan_rew"] == fwd["nan_rew"] == rnd["nan_rew"] == 0)
    # Zero action maps to the middle of [V_MIN, V_MAX], which is NOT zero
    # speed -- it is 0.4 m/s. Standing still would mean the action mapping is
    # wrong, so this check is deliberately "it moves".
    check("zero action creeps forward, not stationary",
          zero["moved"] > 0.5, "moved %.2f m" % zero["moved"])
    check("full throttle travels further than zero action",
          fwd["moved"] > zero["moved"], "%.2f vs %.2f" % (fwd["moved"], zero["moved"]))
    check("driving forward earns more than acting randomly",
          fwd["reward"] > rnd["reward"],
          "%.3f vs %.3f" % (fwd["reward"], rnd["reward"]))
    check("random actions do terminate episodes", rnd["deaths"] > 0)
    check("full throttle does not terminate everything every step",
          fwd["deaths"] < args.num_envs * args.steps * 0.5,
          "%d deaths over %d env-steps" % (fwd["deaths"], args.num_envs * args.steps))

    env.close()
    failed = [r for r in RESULTS if not r[1]]
    print("\n%d checks, %d failed" % (len(RESULTS), len(failed)))
    print("=" * 60)
    print("PHASE 4 ENV GATE: %s" % ("PASS" if not failed else "FAIL"))
    print("=" * 60)
    return 1 if failed else 0


if __name__ == "__main__":
    # Print the traceback BEFORE closing the app. Kit's close() can terminate
    # the process itself, which swallows a pending exception and leaves an
    # exit code of 0 -- a silent failure that looks like a clean run.
    code = 1
    try:
        code = main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        simulation_app.close()
    sys.exit(code)
