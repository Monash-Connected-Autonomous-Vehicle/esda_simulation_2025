#!/usr/bin/env python3
"""Train the course-following policy with PPO.

WHY THIS EXISTS RATHER THAN IsaacLab's OWN train.py
    Isaac Lab's scripts discover tasks inside isaaclab_tasks. An external
    package has to be imported for its gym.register to run, and that import
    can only happen AFTER the simulation app starts, because the task module
    pulls in isaaclab.envs. Passing "esda_isaac:Esda-Course-v0" does not help;
    Isaac Lab splits on the colon and discards the prefix rather than
    importing it. So the import lives here, in the four lines after the app
    launches.

USAGE
    C:\\Users\\Ishaan\\IsaacLab\\isaaclab.bat -p research\\isaac\\scripts\\train.py --num_envs 4096
    ... --max_iterations 50          short run, to check throughput
    ... --resume --load_run <name>   continue, e.g. to advance the curriculum
    ... --viz kit                    watch it. Headless is the default in 3.0,
                                     and --viz takes a backend name, not a flag.
"""
import argparse
import os
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_DIR = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, ISAAC_DIR)
sys.path.insert(0, os.path.join(ISAAC_DIR, "esda_isaac"))

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--task", default="Esda-Course-v0")
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--max_iterations", type=int, default=None)
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--run_name", default=None)
parser.add_argument("--resume", action="store_true")
parser.add_argument("--load_run", default=None)
parser.add_argument("--checkpoint", default=None)
parser.add_argument("--log_root", default=os.path.join(ISAAC_DIR, "logs"))

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
from isaaclab.utils.dict import print_dict  # noqa: E402
from isaaclab.utils.io import dump_yaml  # noqa: E402

import esda_isaac.tasks  # noqa: E402,F401  -- registers the gym ids
from esda_isaac.tasks.course.agents.rsl_rl_ppo_cfg import (  # noqa: E402
    EsdaCoursePPORunnerCfg,
)


def main():
    env_cfg = gym.spec(args.task).kwargs["env_cfg_entry_point"]()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.seed = args.seed

    agent_cfg = EsdaCoursePPORunnerCfg()
    # rsl-rl >= 4 replaced the single `policy` config with separate actor and
    # critic model configs. Isaac Lab ships this migration; without it the
    # runner fails with a bare KeyError: 'class_name'.
    from importlib import metadata

    # Pass the version STRING; the helper parses it itself.
    agent_cfg = handle_deprecated_rsl_rl_cfg(
        agent_cfg, metadata.version("rsl-rl-lib"))
    agent_cfg.device = args.device if args.device else "cuda:0"
    agent_cfg.seed = args.seed
    if args.max_iterations is not None:
        agent_cfg.max_iterations = args.max_iterations
    if args.run_name:
        agent_cfg.run_name = args.run_name

    log_root = os.path.join(args.log_root, agent_cfg.experiment_name)
    os.makedirs(log_root, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = os.path.join(log_root, stamp + ("_" + args.run_name if args.run_name else ""))

    env = gym.make(args.task, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir,
                            device=agent_cfg.device)
    if args.resume:
        run = args.load_run or sorted(os.listdir(log_root))[-1]
        run_dir = os.path.join(log_root, run)
        ckpt = args.checkpoint or sorted(
            f for f in os.listdir(run_dir) if f.startswith("model_"))[-1]
        print("[train] resuming from %s" % os.path.join(run_dir, ckpt))
        runner.load(os.path.join(run_dir, ckpt))

    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)

    print("\n[train] task        : %s" % args.task)
    print("[train] envs        : %d" % args.num_envs)
    print("[train] device      : %s" % agent_cfg.device)
    print("[train] obs / action: %d / %d" % (env_cfg.observation_space,
                                             env_cfg.action_space))
    print("[train] log dir     : %s\n" % log_dir)

    runner.learn(num_learning_iterations=agent_cfg.max_iterations,
                 init_at_random_ep_len=True)
    env.close()
    return 0


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
