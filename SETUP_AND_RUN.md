# ESDA Simulation — Setup and Run Guide

Branch: `yolo-adaptive`

This guide takes a bare Ubuntu 22.04 / WSL2 machine to a driving simulation.
Every troubleshooting entry at the bottom is something that actually broke
during bring-up, not a hypothetical.

---

## 1. What this branch changes

Three files, on top of `esda-jetson-testing`:

| File | Change |
|---|---|
| `src/esda_simulation_2025/scripts/lane_detection_esda.py` | **New.** Subclasses `LaneDetectionNode` and overrides only `detect_white_lanes`. |
| `src/esda_simulation_2025/scripts/ui_launch.py` | Adds an **Adaptive** option to the detector dropdown; side cameras follow the main detector choice; adds the missing shebang. |
| `src/esda_simulation_2025/CMakeLists.txt` | Installs the new script. |

Everything downstream of the mask — Canny, Hough, `filter_valid_lanes`, the
dark-white-dark check, the depth projection, `/lane_obstacles` and the
`/scan_fused` injection — is the unmodified base-class code.

### Why a local threshold

The base detector thresholds grayscale at a fixed level of 130. That works on
even lighting and fails when the lighting is not even. Measured over 395
benchmark frames across four lighting conditions:

| Condition | Fixed threshold | Adaptive |
|---|---|---|
| Even lighting, recall | 0.800 | 0.791 |
| Shadow band, recall | 0.678 | 0.795 |
| Gradient, precision | 0.178 | ~0.79 |

A fixed level loses a third of its precision under a brightness gradient. The
local threshold holds across all four. It also costs 0.9–2.4 ms per frame
against 8.4 ms for the segmentation network — relevant on an Orin NX where
throughput is the whole budget.

Otsu's method scored highest on the clean bench and was the original default.
It was dropped because a global threshold has no notion of "there are no lanes
here": on bare asphalt it split the asphalt itself and flooded the costmap,
driving the robot off course. `lane_detection_esda.py` carries a guard that
rejects any mask covering more than one sixth of the ROI, but adaptive is the
safer default regardless.

### Sensor mount (default changed 2026-10-20)

The default is now **camera on the pole (1.30 m, 15 deg down), LiDAR low on
the chassis (0.385 m)** - `camera_mount:=pole_top lidar_mount:=low`.

Measured over 36 runs, 3 worlds x 4 mount combinations x 3 repeats:

| configuration | igvc | ic | bball | mean rank |
|---|---|---|---|---|
| front cam + pole LiDAR (old default) | 10.5 | 0.0 | 2.3 | 3.00 |
| pole cam + pole LiDAR | 10.1 | 4.6 | 0.0 | 3.33 |
| front cam + low LiDAR | 15.5 | 0.0 | 9.0 | 2.33 |
| **pole cam + low LiDAR (new default)** | 11.3 | **12.4** | **18.8** | **1.33** |

Median furthest-from-start in metres. Two separate effects: LiDAR height
governs whether the robot drives (path length roughly doubles, stalls fall
from 72% to 35%, in both camera positions), camera height governs whether
lanes are seen at all (six consecutive front-camera runs on ic detected zero
segments).

**If you change `camera_mount`, change the detector geometry with it:**

```
pole_top:  -p camera_mount_height:=1.30  -p camera_mount_pitch:=-0.2618
front:     -p camera_mount_height:=0.315 -p camera_mount_pitch:=0.0
```

The pitch sign is **negative** in the detector and **positive** in
`camera.xacro` for the same 15 degrees. The detector rotates rays in the
optical frame (Y down) and solves `t = height / ray_y_rot`, so a downward
tilt is negative there. Get it wrong and the mask still looks perfect in the
preview while `/lane_obstacles` projects to the wrong ground range. The UI
now derives these from its own mount dropdown, so this only bites a bare
`ros2 run`.

Not settled: *why* LiDAR height matters. The obvious explanation, that a
pole-top LiDAR scans over the barrels, is disproved - `min_lidar_clearance_m`
is 0.10-0.96 m for pole configurations and 0.35-0.59 m for low, so all of
them get close returns. The stall pattern points at localisation instead.

---

### Detector methods

`-p detector_method:=` accepts `adaptive` (default), `otsu`, `tophat`,
`value`, `fixed`, `yolo`. `fixed` reproduces the base detector exactly and is
the comparison baseline.

---

## 2. Dependencies

ROS 2 Humble on Ubuntu 22.04. Then:

```bash
sudo apt update
sudo apt install -y \
  libserial-dev \
  xfonts-base \
  xterm \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-image \
  ros-humble-gz-ros2-control

pip3 install -U packaging
pip3 install customtkinter
```

Each of these has a specific failure mode if missing:

- **`libserial-dev`** — `esda_hardware_2025` fails to compile with
  `fatal error: libserial/SerialPort.h`. Simulation does not use this package,
  but colcon aborts the whole build when it fails.
- **`xfonts-base`** — xterm cannot load its default font and may exit silently.
  The UI launches every module inside an xterm, so this looks like "the buttons
  do nothing".
- **`packaging` upgrade** — setuptools calls
  `canonicalize_version(strip_trailing_zero=...)`, which needs packaging ≥ 24.0.
  Older versions fail the build with a `TypeError`.
- **`customtkinter`** — the UI will not start without it.

---

## 3. Build

**Put the workspace on the Linux filesystem, not under `/mnt/c`.** Builds
through DrvFs are several times slower, and Windows and Linux disagree about
file permissions and timestamps in ways that waste hours.

```bash
cd ~
git clone -b yolo-adaptive https://github.com/Monash-Connected-Autonomous-Vehicle/esda_simulation_2025.git esda_ws
cd ~/esda_ws
colcon build --symlink-install
```

Add the workspace to your shell:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo '[ -f ~/esda_ws/install/setup.bash ] && source ~/esda_ws/install/setup.bash' >> ~/.bashrc
```

**Keep exactly one ESDA workspace on the machine.** If you have several copies
(ZIP downloads, old clones), `.bashrc` will source one of them and
`ros2 run` will silently use the wrong one. Verify with:

```bash
ros2 pkg prefix esda_simulation_2025
```

That must print the workspace you intend to use. `No executable found` almost
always means a different workspace is active, not that the file is missing.

---

## 4. Running — the UI

```bash
ros2 run esda_simulation_2025 ui_launch.py
```

**Press the buttons in this order. The order is not optional.**

1. **Launch Simulation** — Gazebo, the robot, the bridges
2. **Launch Lane Detection** — with the dropdown set to **Adaptive**
3. **Launch SLAM** (or AMCL for a known map)
4. **Launch Nav2**
5. **Launch RViz2**
6. **Waypoint Nav** — sends goals; this is what makes it drive

### Why step 2 cannot be skipped

The "Enable Lane Detection" checkbox does **not** start the detector. It
selects which scan topic the navigation stack subscribes to:

```python
return "/scan_fused" if self.lane_detection_var.get() else "/scan"
```

`/scan_fused` is published by the lane detection node and by nothing else. It
is wired into `mapper_params_online_async.yaml`, the Nav2 voxel and obstacle
layers, the collision monitor, and `view_bot.rviz`.

So with the checkbox ticked and the detector not running, SLAM and Nav2
subscribe to a topic no one publishes. No scans, no map, empty costmap,
nothing to plan against — **the robot sits still and nothing reports an
error.** Either press Launch Lane Detection, or untick the box to fall back to
raw `/scan`.

Confirm the chain before going further:

```bash
ros2 topic hz /scan_fused     # expect ~10 Hz
```

### Known UI issues

- The UI's shutdown routine kills `lane_detection.py`, `lane_detection_FCN.py`
  and `lane_detection_twinlite.py` but **not** `lane_detection_esda.py`. Run
  `pkill -9 -f lane_detection_esda.py` between runs or the old node fights the
  new one for the camera topic.
- Gazebo warns `Unable to find the following fonts: Ubuntu Mono` on startup.
  Cosmetic; click OK.
- `Failed to get Ogre item for [model://...stl]` lines are resource-path
  failures affecting visuals only. The physics and collision geometry load.
- `ign_ros2_control` → `gz_ros2_control` is a rename warning, not a failure.

---

## 5. Running — plain terminal

```bash
# terminal 1
ros2 launch esda_simulation_2025 launch_sim.launch.py

# terminal 2
ros2 run esda_simulation_2025 lane_detection_esda.py \
  --ros-args -p detector_method:=adaptive -p show_visualization:=true

# terminal 3
ros2 launch esda_simulation_2025 online_async_launch.py use_sim_time:=true

# terminal 4
ros2 launch esda_simulation_2025 navigation_launch.py \
  use_sim_time:=true map_subscribe_transient_local:=true

# terminal 5
rviz2 -d src/esda_simulation_2025/config/view_bot.rviz
```

Source `/opt/ros/humble/setup.bash` and `install/setup.bash` in every terminal.

---

## 6. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `No executable found` | A different workspace is sourced | `ros2 pkg prefix esda_simulation_2025`; fix `.bashrc` |
| `Exec format error` | Script has no shebang | Add `#!/usr/bin/env python3` as line 1 |
| `fatal error: libserial/SerialPort.h` | `libserial-dev` missing | `sudo apt install -y libserial-dev` |
| `canonicalize_version() got an unexpected keyword argument` | packaging < 24.0 | `pip3 install -U packaging` |
| `No module named 'customtkinter'` | UI dependency missing | `pip3 install customtkinter` |
| `xterm: cannot load font` and buttons do nothing | `xfonts-base` missing | `sudo apt install -y xfonts-base` |
| Robot never moves, no errors | `/scan_fused` has no publisher | Launch Lane Detection, or untick Enable Lane Detection |
| Nav2 reports huge timeouts; `ros2 topic hz /clock` shows ±3600 s | Host clock wrong, two time sources fighting | See below |
| `make: warning: Clock skew detected` | Files have future mtimes | Fix the clock, then `rm -rf build install log` and rebuild |
| Robot drives off the course into open ground | Lane mask flooding the costmap | Use `adaptive`, not `otsu`, on surfaces without lane markings |

### The clock problem

WSL2 takes its UTC from the Windows host. If the host clock is wrong, every
Linux VM on the machine inherits the error, and if a time daemon inside WSL
also syncs to a real time server the two fight — stepping the clock back and
forth. That corrupts ROS message timestamps and breaks TF, SLAM and Nav2,
because transform tolerances are measured in tenths of a second.

Diagnose:

```bash
date -u                                   # WSL's view
powershell.exe -NoProfile -Command "(Get-Date).ToUniversalTime().ToString('u')"
```

Both must match real UTC. If Windows is wrong, fix it at the source — the
Windows Time service is often disabled:

```powershell
# Administrator PowerShell
Set-Service w32time -StartupType Automatic
Start-Service w32time
w32tm /resync /force
```

Also check Settings → Time & language that "Adjust for daylight saving time
automatically" is on. A wrong DST setting and a wrong clock can cancel out in
the local-time display while leaving UTC an hour off — which looks fine on the
taskbar and breaks everything underneath.

Confirm it holds, not just that it reads right once:

```bash
for i in $(seq 1 90); do date +%s; sleep 1; done | \
  awk 'NR>1 && $1-p!=1 {print "jump "($1-p)"s"} {p=$1} END{print "done"}'
```

Any output other than `done` means the clock is still stepping.

---

## 7. Benchmarks

`research/` holds the evaluation code behind the numbers above:

- `vision/eval_recall.py` — recall and precision per detector and lighting
  (`--detector classic|yolo`, `--method`, `--perturb none|bright|dark|gradient|shadow`)
- `vision/backtest_detectors.py` — scores `fixed` against `adaptive`, `otsu`
  and `tophat` across five lightings
- `vision/lane_classic.py` — the threshold implementations, with self-tests
- `vision/pose_bench.py` — teleport benchmark; measures perception against
  commanded poses rather than integrated odometry

Measuring against integrated odometry rather than commanded poses was what made
recall look like 0.174. The same detector measured correctly scores 0.492.
If a perception number looks impossibly bad, check what it is being compared
against before changing the detector.
