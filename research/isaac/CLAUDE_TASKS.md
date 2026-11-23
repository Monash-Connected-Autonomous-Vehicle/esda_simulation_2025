# Isaac runbook — build, train, and check it worked

Everything here runs on **Windows**, through Isaac Lab's launcher. `ISAACLAB`
below means `C:\Users\Ishaan\IsaacLab\isaaclab.bat`. Run from the repo root.

Shut WSL down first for long runs — `.wslconfig` reserves 24 GB and 12 of 16
cores, and Isaac's main thread wants P-cores:

```powershell
wsl --shutdown
```

---

## 0. Once per machine

```powershell
C:\isaac-sim\python.bat -m pip install xacro
```

```powershell
C:\isaac-sim\python.bat -m pip install -e research\isaac\esda_isaac
```

Check Isaac Lab and the GPU are healthy. This is the one script that proves
the pairing works and **terminates** — `scripts/tutorials/00_sim/create_empty.py`
never exits, which looks exactly like a hang:

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\gate_check.py
```

> **eGPU:** power the DEG1 dock *before* booting. If the card was cold at boot
> it is simply absent, and the failure looks like a software problem.

---

## 1. Build the assets

Regenerate in this order. Each is fast and each prints its own checks.

```powershell
C:\isaac-sim\python.bat tools\export_urdf_for_isaac.py
```

Watch for: total **56.4 kg**, and a drive/caster load split around **72/28**.
A split under 50% means the centre of mass has moved behind the axle and the
robot will struggle for traction — fix `com_xyz` in `config/robot_mass.yaml`.

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\convert_robot.py
```

Watch for: 5 rigid bodies, 4 joints, and friction bound to **1 caster**
collider. If it says anything other than one, the caster is wearing
wheel-grade friction and the robot will fight every turn.

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\build_course_usd.py
```

Watch for: 34 barrels, 4 barricades, 2 ramps, and `22 inside / 7 edge /
5 beyond` on the corridor check. Then **look at**
`usd/course/snapshots/course_overlay.png` — barricades should sit on the four
black strokes and the ramps between the two top-centre lines.

---

## 2. Check the vehicle before you train it

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\drive_check.py
```

Must print `PHASE 1 GATE: PASS`. It drives a 20 m square and checks mass,
speed tracking, odometry and that the robot is still upright.

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\env_check.py
```

Must print `PHASE 4 ENV GATE: PASS`. Builds the RL environment and drives
zero, forward and random policies through it.

```powershell
python3 tools\validate_isaac_obs.py
```

101 checks, no Isaac needed. Run this after touching `obs_core.py`,
`scan_sim.py` or `scan_sim_torch.py` — it is what stops the trainer and the
ROS node drifting apart.

---

## 3. Drive it yourself

**Do this before training.** A reward curve cannot tell you whether the
vehicle feels right or whether its observation is sane; two minutes at the
wheel tells you both.

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\teleop.py --viz
```

Keys are the same as `scripts/teleop_wasd.py`, so the same fingers drive the
robot in Gazebo, in Isaac and on the real vehicle:

| key | action |
|---|---|
| `w` / `s` | forward / reverse |
| `a` / `d` | turn left / right |
| `x` | stop |
| `u` / `j` | max speed ±10% |
| `i` / `k` | max turn rate ±10% |
| `r` | reset to the course start |
| `p` | full observation summary |
| `q` | quit |

Click the viewport first so it has keyboard focus. The readout is:

```
v +0.60/+0.60 m/s  w +0.00/+0.00 rad/s | lateral -0.13 (half 1.44) | obstacle 2.80  scan 2.45
   actual/commanded    actual/commanded   offset from centreline      nearest    nearest
                                          and the corridor half-width  obstacle   scan return
```

**What to look for**

- *Actual speed should track commanded.* A persistent gap means slip —
  check wheel friction.
- *`lateral` should change smoothly* as you steer, and `half` should be
  roughly 1.5 m on the real course.
- *`scan` should be larger than `obstacle` when the nearest thing is a lane
  line*, because the camera cone cannot see paint beside the robot — only
  ahead. If `scan` is ever much smaller than you would expect from what is on
  screen, the observation is wrong and training will not fix it.
- *Press `p` on a straight* and check `beams under the robot radius` is **0**.
  Anything else means the policy is being told it is about to crash.
- *Drive over the ramp* (top-centre of the course) and watch it stay upright.

---

## 4. Train

Training is yours to run. Start with a short run to confirm throughput:

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\train.py --num_envs 1024 --max_iterations 20
```

Then the real run:

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\train.py --num_envs 4096 --run_name v1 --seed 1
```

Watch it in TensorBoard:

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p -m tensorboard.main --logdir research\isaac\logs\esda_course
```

Resume:

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\train.py --num_envs 4096 --resume --load_run v1
```

**The curriculum runs itself**, printing `[curriculum] stage N: X of Y
courses, noise xZ` as it advances. Stage 0 is four easy layouts at quarter
noise, stage 1 opens all ten at full noise. Thresholds are in *environment
steps*, so they mean the same thing at any `--num_envs`. Pass
`curriculum=False` in the config to disable.

**Reading the numbers**

| signal | what it means |
|---|---|
| reward flat near zero past ~200 iterations | progress term too weak against the penalties |
| episode length pinned at 320 with low reward | learned to stand still; raise `rew_time` |
| episode length very short (< 20) | something terminates instantly — run `env_check.py` |
| reward falls as episode length rises | shaping is mis-scaled; progress should dominate |
| iterations/second sags mid-run | thermal or power on the eGPU, not the code |

Do **not** add `--enable_cameras`. Nothing here renders during training, and
it would push frames over the eGPU's PCIe x4 link every step.

---

## 5. Check whether it worked

### Watch it drive

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\play.py --load_run v1 --num_envs 1 --viz
```

### Measure it

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\play.py --load_run v1 --num_envs 32 --episodes 5
```

```
episodes finished     : 160
best lap fraction     : mean 0.86, max 1.00
mean speed            : 0.74 m/s
lane crossings/episode: 0.04
collisions/episode    : 0.09
timeouts/episode      : 0.21
```

Targets from the plan: **≥80% lap completion** on held-out courses and **zero
lane crossings** on a clean run of the real course.

This runs on `Esda-Course-Play-v0`, which is the **real** course — held out of
training on purpose, because 34 memorisable barrel positions would otherwise
turn the score into a recall test.

### The robustness sweep — the one that predicts the real robot

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\play.py --load_run v1 --num_envs 32 --noise 1.0
```

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\play.py --load_run v1 --num_envs 32 --noise 2.0
```

`--noise 0` is a clean sensor, `1.0` is as degraded as training was. Real lane
detection measures 0.92 recall on rejoining fragments
(`research/vision/README_VISION.md`), so a policy that collapses between 0 and
1 will not survive the real `/scan_fused`. A policy that holds up at 2.0 is
worth putting on the robot.

### Export for deployment

```powershell
C:\Users\Ishaan\IsaacLab\isaaclab.bat -p research\isaac\scripts\play.py --load_run v1 --export
```

Writes `policy.onnx` and `policy.pt` into `logs/esda_course/<run>/exported/`,
with the observation normaliser baked in. ONNX is the one to deploy: the ROS
side is Python 3.10 with `numpy<2` pinned for `cv_bridge`, and dragging torch
2.10 / cu128 / py3.12 into Humble is not worth it.

---

## Known gaps

- **The masses are estimates.** Weigh the vehicle and replace them in
  `config/robot_mass.yaml`. Nothing trained against the current numbers is
  trustworthy until that happens — this is the largest open assumption in the
  stack.
- **The ramp is not in the training scene.** It exists in the course USD and
  you can drive it in teleop, but training never meets it. The policy cannot
  *see* it either way (a 0.05 m slab is below the lidar), so what it would
  learn is pitch-disturbance robustness, not a decision.
- **Yaw slip differs from Gazebo.** Isaac's wheels genuinely slip through
  standing turns; Gazebo's `diff_drive_controller` runs `open_loop: true` and
  integrates the command without ever consulting the ground. A policy trained
  here meets a slightly different vehicle there.
- **Reward weights are reasoned, not tuned.** They were scaled so progress
  reads as speed in m/s and terminal penalties cost about a sixth of a perfect
  episode. That is arithmetic, not evidence.
- **Phase 6 is not written.** No `rl_policy_node.py` yet, so the exported
  ONNX has nothing to run it in ROS.
