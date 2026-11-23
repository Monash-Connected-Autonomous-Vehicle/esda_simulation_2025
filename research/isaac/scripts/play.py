#!/usr/bin/env python3
"""Run a trained policy and measure whether it actually drives the course.

WHAT IT REPORTS, AND WHY NOT REWARD
    Reward is a training signal, not a result. It mixes progress with shaping
    terms and cannot be compared against the existing Nav2 stack. These can:

        laps completed      fraction of episodes that got all the way round
        distance per episode along the centreline, in metres
        mean speed          m/s while driving
        lane crossings      per episode. An IGVC fault, so it ends the run
        collisions          per episode
        timeouts            ran out of clock without finishing or failing

    Measured on the REAL course, which is held out of training -- see
    course_env_cfgs.EsdaCoursePlayEnvCfg. A number from a course the policy
    trained on would measure recall.

ON THE ROBUSTNESS SWEEP
    --noise scales the observation degradation. The play config sets it to
    zero so the headline number reflects the policy. Re-running at 1.0 and
    above answers a different and more important question: whether it survives
    a sensor as bad as the real one. research/vision measures 0.92 recall on
    rejoining lane fragments, so a policy that collapses between 0 and 1 will
    not survive real lane detection.

USAGE
    C:\\Users\\Ishaan\\IsaacLab\\isaaclab.bat -p research\\isaac\\scripts\\play.py --load_run <run>
    ... --viz kit             watch it (--viz needs a backend, e.g. kit)
    ... --num_envs 32         more episodes, better statistics
    ... --noise 1.5           robustness sweep
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_DIR = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, ISAAC_DIR)
sys.path.insert(0, os.path.join(ISAAC_DIR, "esda_isaac"))

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--task", default="Esda-Course-Play-v0")
parser.add_argument("--num_envs", type=int, default=32)
parser.add_argument("--load_run", default=None,
                    help="run folder under research/isaac/logs/esda_course; "
                         "defaults to the most recent")
parser.add_argument("--checkpoint", default=None)
parser.add_argument("--episodes", type=int, default=3,
                    help="episodes per environment to average over")
parser.add_argument("--noise", type=float, default=0.0,
                    help="observation noise multiplier for the robustness sweep")
parser.add_argument("--log_root", default=os.path.join(ISAAC_DIR, "logs"))
parser.add_argument("--export", action="store_true",
                    help="also write policy.onnx and policy.pt beside the run")

from isaaclab.app import AppLauncher  # noqa: E402

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlVecEnvWrapper,
    handle_deprecated_rsl_rl_cfg,
)

import esda_isaac.tasks  # noqa: E402,F401  -- registers the gym ids
from esda_isaac.tasks.course.agents.rsl_rl_ppo_cfg import (  # noqa: E402
    EsdaCoursePPORunnerCfg,
)


def resolve_run(log_root):
    root = os.path.join(log_root, "esda_course")
    if not os.path.isdir(root):
        sys.exit("no runs under %s -- train something first" % root)
    run = args.load_run or sorted(os.listdir(root))[-1]
    run_dir = os.path.join(root, run)
    ckpts = sorted(f for f in os.listdir(run_dir) if f.startswith("model_"))
    if not ckpts:
        sys.exit("no model_*.pt in %s" % run_dir)
    return run_dir, os.path.join(run_dir, args.checkpoint or ckpts[-1])


def main():
    env_cfg = gym.spec(args.task).kwargs["env_cfg_entry_point"]()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.curriculum = False          # evaluate the full task, not a stage
    # Scale the TRAINING noise levels, so --noise 1.0 means "as degraded as
    # training was" and 0.0 means a clean sensor. The play config zeroes them,
    # so the baseline has to come from the training config.
    from esda_isaac.tasks.course.course_env import EsdaCourseEnvCfg
    base = EsdaCourseEnvCfg()
    for attr in ("noise_dropout", "noise_lane_dropout", "noise_spurious",
                 "noise_range_bias"):
        setattr(env_cfg, attr, getattr(base, attr) * args.noise)

    run_dir, ckpt = resolve_run(args.log_root)
    print("\n[play] task      : %s" % args.task)
    print("[play] checkpoint: %s" % ckpt)
    print("[play] noise x   : %.2f" % args.noise)

    agent_cfg = EsdaCoursePPORunnerCfg()
    agent_cfg.device = args.device if args.device else "cuda:0"
    from importlib import metadata
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg,
                                             metadata.version("rsl-rl-lib"))

    env = gym.make(args.task, cfg=env_cfg, render_mode=None)
    wrapped = RslRlVecEnvWrapper(env)
    runner = OnPolicyRunner(wrapped, agent_cfg.to_dict(), log_dir=None,
                            device=agent_cfg.device)
    runner.load(ckpt)
    policy = runner.get_inference_policy(device=agent_cfg.device)

    if args.export:
        # rsl-rl >= 4.0.0 exports from the RUNNER, which bakes the observation
        # normaliser in. The older isaaclab_rl export_policy_as_* helpers want
        # the bare network and fail here with "'PPO' object has no attribute
        # 'policy'". Deploying a policy without its normaliser would be worse
        # than failing: it would run, and drive badly.
        out = os.path.join(run_dir, "exported")
        os.makedirs(out, exist_ok=True)
        runner.export_policy_to_jit(path=out, filename="policy.pt")
        runner.export_policy_to_onnx(path=out, filename="policy.onnx")
        print("[play] exported  : %s" % out)

    core = env.unwrapped
    n = core.num_envs
    dev = core.device
    laps = torch.zeros(n, device=dev)
    speed_sum = torch.zeros(n, device=dev)
    steps = torch.zeros(n, device=dev)
    crossings = collisions = timeouts = finished = 0

    obs = wrapped.get_observations()
    if isinstance(obs, tuple):          # arity differs by version
        obs = obs[0]
    budget = args.episodes * int(core.max_episode_length)
    with torch.inference_mode():
        for _ in range(budget):
            obs, _, dones, extras = wrapped.step(policy(obs))
            speed_sum += core.twist[:, 0]
            steps += 1
            if dones.any():
                idx = dones.nonzero(as_tuple=False).flatten()
                crossings += int(core._crossed[idx].sum())
                collisions += int(core._collided[idx].sum())
                lap_frac = (core.lap_progress[idx]
                            / core.bank_length[core.course_idx[idx]])
                laps[idx] = torch.maximum(laps[idx], lap_frac)
                done_timeout = ~(core._crossed[idx] | core._collided[idx])
                timeouts += int(done_timeout.sum())
                finished += len(idx)

    mean_speed = float((speed_sum / steps.clamp(min=1)).mean())
    print("\n  episodes finished     : %d" % finished)
    print("  best lap fraction     : mean %.2f, max %.2f"
          % (float(laps.mean()), float(laps.max())))
    print("  mean speed            : %.2f m/s" % mean_speed)
    if finished:
        print("  lane crossings/episode: %.2f" % (crossings / finished))
        print("  collisions/episode    : %.2f" % (collisions / finished))
        print("  timeouts/episode      : %.2f" % (timeouts / finished))
    print("\n  Targets from the plan: >=80% lap completion on held-out courses,")
    print("  zero lane crossings on a clean run of the real course.")

    env.close()
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
