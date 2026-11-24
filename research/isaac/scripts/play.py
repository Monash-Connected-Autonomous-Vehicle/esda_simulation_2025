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
import math
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
parser.add_argument("--episode_s", type=float, default=None,
                    help="episode length in seconds. The training default of "
                         "40 s covers ~40 m at 1 m/s, so a 135.7 m lap cannot "
                         "complete; set ~180 to measure laps.")
parser.add_argument("--episodes", type=int, default=3,
                    help="episodes per environment to average over")
parser.add_argument("--noise", type=float, default=0.0,
                    help="observation noise multiplier for the robustness sweep")
parser.add_argument("--log_root", default=os.path.join(ISAAC_DIR, "logs"))
parser.add_argument("--diagnose", action="store_true",
                    help="record where each collision happens and which "
                         "obstacle caused it")
parser.add_argument("--video", type=float, default=0.0, metavar="SECONDS",
                    help="record this many seconds of env 0 to an mp4 with a "
                         "chase camera. Needs --enable_cameras.")
parser.add_argument("--video_fps", type=int, default=30)
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

import obs_core as OC  # noqa: E402
import esda_isaac.tasks  # noqa: E402,F401  -- registers the gym ids
from esda_isaac.tasks.course.agents.rsl_rl_ppo_cfg import (  # noqa: E402
    EsdaCoursePPORunnerCfg,
)


def _iteration(name):
    """The number in model_<N>.pt, for sorting.

    Lexicographic sort is wrong here and wrong QUIETLY: among 31 checkpoints
    from a 3000-iteration run it picks model_900.pt as the newest, because
    "900" > "2900" as text. That evaluates a policy from a third of the way
    through training and reports it as the final result.
    """
    try:
        return int(name[len("model_"):].split(".")[0])
    except ValueError:
        return -1


def resolve_run(log_root):
    """Find a run directory and its latest checkpoint.

    train.py names runs <timestamp>_<run_name>, so --load_run v1 will not match
    exactly. Accept a substring and take the most recent match, which is what
    someone typing the name they passed to --run_name expects.
    """
    root = os.path.join(log_root, "esda_course")
    if not os.path.isdir(root):
        sys.exit("no runs under %s -- train something first" % root)

    runs = sorted((d for d in os.listdir(root)
                   if os.path.isdir(os.path.join(root, d))),
                  key=lambda d: os.path.getmtime(os.path.join(root, d)))
    if not runs:
        sys.exit("no run directories under %s" % root)

    if not args.load_run:
        run = runs[-1]
    elif args.load_run in runs:
        run = args.load_run
    else:
        matches = [d for d in runs if args.load_run in d]
        if not matches:
            sys.exit("no run matching %r under %s\navailable:\n  %s"
                     % (args.load_run, root,
                        "\n  ".join(reversed(runs))))
        run = matches[-1]
        print("[play] run       : %s (matched %r)" % (run, args.load_run))

    run_dir = os.path.join(root, run)
    ckpts = sorted((f for f in os.listdir(run_dir) if f.startswith("model_")),
                   key=_iteration)
    if not ckpts:
        sys.exit("no model_*.pt in %s" % run_dir)
    chosen = args.checkpoint or ckpts[-1]
    print("[play] iterations: %d checkpoints, newest is %s"
          % (len(ckpts), ckpts[-1]))
    return run_dir, os.path.join(run_dir, chosen)


def report_collisions(where, core):
    """Where collisions happen, and what the robot hit.

    course_bank.igvc_course packs barrels first and barricades after, so the
    obstacle index says which kind it was. That distinction is the point:
    barricades are 3.0 x 0.5 m boxes flattened into a single 0.25 m disc, so
    if they dominate this table the obstacle model is the problem rather than
    the policy.
    """
    import collections

    import yaml
    with open(os.path.join(ISAAC_DIR, "config", "course_obstacles.yaml")) as fh:
        obst = yaml.safe_load(fh) or {}
    n_barrels = len(obst.get("barrels") or [])
    barricades = obst.get("barricades") or []

    print("\n  --- where it crashes (%d collisions) ---" % len(where))

    bins = [0] * 10
    for frac, _, _ in where:
        bins[min(9, int(frac * 10))] += 1
    print("  by position around the lap:")
    peak = max(bins) or 1
    for i, count in enumerate(bins):
        print("    %3d-%3d%%  %-30s %d"
              % (i * 10, (i + 1) * 10, "#" * int(28 * count / peak), count))

    tally = collections.Counter(o for _, o, _ in where)
    print("  worst obstacles:")
    for obstacle, count in tally.most_common(8):
        if obstacle < n_barrels:
            x, y, r = obst["barrels"][obstacle]
            kind = "barrel  r=%.2f" % r
        else:
            row = barricades[obstacle - n_barrels]
            x, y = row[0], row[1]
            kind = "BARRICADE %.1f x %.1f m, modelled as r=%.2f disc" % (
                row[3], row[4], max(0.5, row[4]) / 2.0)
        print("    %5.1f%%  obstacle %2d at (%+7.2f,%+7.2f)  %s"
              % (100.0 * count / len(where), obstacle, x, y, kind))

    n_barricade_hits = sum(1 for _, o, _ in where if o >= n_barrels)
    print("  barricades account for %.0f%% of collisions (%d of %d)"
          % (100.0 * n_barricade_hits / len(where), n_barricade_hits, len(where)))


def record(core, wrapped, policy, run_dir):
    """Film env 0 from a chase camera. Worth watching before trusting a number.

    The environment normally has no course geometry -- the course is tensors --
    so EsdaCoursePlayEnvCfg.show_course spawns the course USD. Its obstacle
    colliders are turned off, because the course tensors already decide
    contact; its RAMP colliders stay on, because height has no tensor and
    without them the robot drives through the ramp instead of over it. So what
    you see is what the policy drove, including the climb.
    """
    import cv2
    import numpy as np
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import Camera, CameraCfg

    cam = Camera(CameraCfg(prim_path="/World/envs/env_0/ChaseCam",
                           update_period=0.0, height=720, width=1280,
                           data_types=["rgb"],
                           spawn=sim_utils.PinholeCameraCfg(
                               focal_length=20.0, clipping_range=(0.1, 400.0))))
    core.sim.reset()
    obs = wrapped.get_observations()
    if isinstance(obs, tuple):
        obs = obs[0]

    out_dir = os.path.join(run_dir, "video")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "lap.mp4")
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"),
                             args.video_fps, (1280, 720))

    control_dt = core.cfg.sim.dt * core.cfg.decimation
    steps = int(round(args.video / control_dt))
    print("[play] recording %.0f s (%d control steps) -> %s"
          % (args.video, steps, path))

    frames = []
    with torch.inference_mode():
        for i in range(steps):
            obs, _, dones, _ = wrapped.step(policy(obs))

            pos = core.robot.data.root_pos_w[0]
            q = core.robot.data.root_quat_w[0]
            x, y, z, w = (float(v) for v in q)          # XYZW
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            # Behind and above, looking at the robot.
            eye = torch.tensor([[float(pos[0]) - 6.0 * math.cos(yaw),
                                 float(pos[1]) - 6.0 * math.sin(yaw),
                                 float(pos[2]) + 3.5]], device=cam.device)
            tgt = torch.tensor([[float(pos[0]) + 2.0 * math.cos(yaw),
                                 float(pos[1]) + 2.0 * math.sin(yaw),
                                 float(pos[2])]], device=cam.device)
            cam.set_world_poses_from_view(eye, tgt)
            core.sim.render()
            cam.update(core.cfg.sim.dt)

            rgb = cam.data.output["rgb"][0].detach().cpu().numpy()
            if rgb.dtype != np.uint8:
                rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
            bgr = cv2.cvtColor(rgb[..., :3], cv2.COLOR_RGB2BGR)

            lap = float(core.lap_progress[0])
            cv2.rectangle(bgr, (0, 0), (1280, 40), (0, 0, 0), -1)
            cv2.putText(bgr, "t=%5.1fs  %5.1f m  v=%.2f m/s%s"
                        % (i * control_dt, lap,
                           float(core.twist[0, 0]),
                           "   COLLISION" if bool(dones[0]) else ""),
                        (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            writer.write(bgr)
            if i % max(1, steps // 6) == 0:
                frames.append(cv2.resize(bgr, (640, 360)))

    writer.release()
    if frames:
        rows = [np.hstack(frames[j:j + 3]) for j in range(0, min(6, len(frames)), 3)]
        width = max(r.shape[1] for r in rows)
        rows = [np.pad(r, ((0, 0), (0, width - r.shape[1]), (0, 0))) for r in rows]
        sheet = os.path.join(out_dir, "lap_filmstrip.png")
        cv2.imwrite(sheet, np.vstack(rows))
        print("[play] filmstrip : %s" % sheet)
    print("[play] video     : %s" % path)
    return 0


def main():
    env_cfg = gym.spec(args.task).kwargs["env_cfg_entry_point"]()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.curriculum = False          # evaluate the full task, not a stage
    if args.episode_s:
        env_cfg.episode_length_s = args.episode_s
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

        # The deployment contract. rl_policy_node.py refuses to run a policy
        # whose meta disagrees with the obs_core it imports, which is the only
        # thing standing between a changed constant and a robot driving on
        # confident, wrong velocities.
        import yaml
        meta_path = os.path.join(out, "policy_meta.yaml")
        with open(meta_path, "w") as fh:
            yaml.safe_dump(OC.policy_meta(), fh, sort_keys=True)
        print("[play] exported  : %s" % out)
        print("[play] contract  : %s" % meta_path)

    core = env.unwrapped
    if args.video > 0.0:
        return record(core, wrapped, policy, run_dir)
    n = core.num_envs
    dev = core.device
    # BEST per environment, and the running totals over every episode. Only
    # the bests were kept at first and they were printed as "mean ... max",
    # which made the headline read as typical behaviour when it was the best
    # of each environment's attempts -- it reported 1.33 laps and 181.9 m on
    # a run where half the episodes ended in a collision, and 181.9 m is
    # above the 180 m the clock allows. Keep both and label them.
    laps = torch.zeros(n, device=dev)        # best lap fraction, per env
    metres = torch.zeros(n, device=dev)      # best distance, per env
    laps_sum = metres_sum = 0.0              # over every finished episode
    running_frac = torch.zeros(n, device=dev)
    running_m = torch.zeros(n, device=dev)
    speed_sum = torch.zeros(n, device=dev)
    steps = torch.zeros(n, device=dev)
    crossings = collisions = timeouts = finished = 0
    where = []

    obs = wrapped.get_observations()
    if isinstance(obs, tuple):          # arity differs by version
        obs = obs[0]
    budget = args.episodes * int(core.max_episode_length)
    with torch.inference_mode():
        for _ in range(budget):
            # Sample progress BEFORE stepping. _reset_idx zeroes lap_progress,
            # and by the time step() returns the done environments have already
            # been reset -- reading it afterwards reports 0 for every episode
            # that finished, which is exactly what it did.
            running_m = torch.maximum(running_m, core.lap_progress)
            running_frac = torch.maximum(
                running_frac,
                core.lap_progress / core.bank_length[core.course_idx])

            # Same reason: the robot's position and the course sample it was
            # nearest are both overwritten by the reset, so a collision has to
            # be located from the state it happened in, not the state after.
            if args.diagnose:
                prev_idx = core._idx.clone()
                prev_xy = (core.robot.data.root_pos_w[:, :2] - core.env_xy).clone()
                prev_course = core.course_idx.clone()

            obs, _, dones, extras = wrapped.step(policy(obs))
            speed_sum += core.twist[:, 0]
            steps += 1
            if dones.any():
                idx = dones.nonzero(as_tuple=False).flatten()
                crossings += int(core._crossed[idx].sum())
                collisions += int(core._collided[idx].sum())
                laps_sum += float(running_frac[idx].sum())
                metres_sum += float(running_m[idx].sum())
                laps[idx] = torch.maximum(laps[idx], running_frac[idx])
                metres[idx] = torch.maximum(metres[idx], running_m[idx])
                running_frac[idx] = 0.0
                running_m[idx] = 0.0
                done_timeout = ~(core._crossed[idx] | core._collided[idx])
                timeouts += int(done_timeout.sum())
                finished += len(idx)

                if args.diagnose:
                    hit = idx[core._collided[idx]]
                    for e in hit.tolist():
                        c = int(prev_course[e])
                        discs = core.bank_discs[c]
                        radii = core.bank_radii[c]
                        gap = torch.linalg.norm(discs - prev_xy[e][None, :], dim=1) - radii
                        gap = torch.where(radii > 0, gap, torch.full_like(gap, 1e3))
                        obstacle = int(gap.argmin())
                        where.append((int(prev_idx[e]) / core.n_samples,
                                      obstacle, float(gap.min())))

    mean_speed = float((speed_sum / steps.clamp(min=1)).mean())
    lap_m = float(core.bank_length[core.course_idx].float().mean())
    print("\n  episodes finished     : %d" % finished)
    if finished:
        print("  distance per episode  : mean %.1f m   (best env %.1f m)"
              % (metres_sum / finished, float(metres.max())))
        print("  lap fraction          : mean %.2f      (best env %.2f)   "
              "lap is %.1f m" % (laps_sum / finished, float(laps.max()), lap_m))
    print("  episode length        : %.0f s -- at 1 m/s that bounds a lap at "
          "%.2f" % (env_cfg.episode_length_s,
                    env_cfg.episode_length_s / lap_m))
    print("  mean speed            : %.2f m/s" % mean_speed)
    if finished:
        print("  lane crossings/episode: %.2f" % (crossings / finished))
        print("  collisions/episode    : %.2f" % (collisions / finished))
        print("  timeouts/episode      : %.2f" % (timeouts / finished))
        # PER DISTANCE, because per-episode figures do not survive a change
        # of episode_length_s: the 40 s and 180 s runs of the same policy
        # reported 0.43 and 0.50 collisions per episode while the collision
        # rate per lap had in fact fallen from 1.46 to 0.38. Compare these.
        if metres_sum > 0:
            print("  --")
            print("  collisions per 100 m  : %.2f   (1 per %.0f m)"
                  % (100.0 * collisions / metres_sum,
                     metres_sum / collisions if collisions else float("inf")))
            print("  collisions per lap    : %.2f"
                  % (collisions / (metres_sum / lap_m)))
            print("  clean episodes        : %.0f%% (no collision, no crossing)"
                  % (100.0 * timeouts / finished))
    if args.diagnose and where:
        report_collisions(where, core)

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
