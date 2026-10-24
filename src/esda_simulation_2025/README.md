## 2025 ESDA Vehicle Simulation

A complete ROS2 Humble autonomous navigation system with advanced lane detection, SLAM mapping, and autonomous navigation capabilities for IGVC-style competitions.

## 🚗 Features

- **🎮 Graphical Simulation Manager** - Easy-to-use GUI for controlling all simulation components
- **🛣️ Advanced Lane Detection** - Camera-based white lane marking detection with costmap integration
- **🗺️ SLAM Mapping** - Real-time environment mapping with SLAM Toolbox
- **🧭 Autonomous Navigation** - Nav2 with optimized MPPI controller for differential drive robots
- **👁️ Multi-Camera System** - Stereo cameras + depth sensing for 3D lane detection
- **🎯 Purple Lane Visualization** - Custom colored obstacles in RViz for easy identification
- **⚡ Optimized Performance** - Tuned for stable 8Hz control with tight space navigation
- **🏁 Autonomous IGVC Runs** - `waypoint_navigator_recommendation.py` follows lanes, routes around barrels and heads for GPS waypoints
- **🌳 Three IGVC Worlds** - Bare track, campus, and a competition-style car park

---

## 📋 Table of Contents

1. [Setup & Installation](#1-setup--installation)
2. [Launching the Simulation](#2-launching-the-simulation)
3. [Autonomous IGVC Run (waypoint_navigator_recommendation.py)](#3-autonomous-igvc-run-waypoint_navigator_recommendationpy)
4. [System Architecture](#4-system-architecture)
5. [Lane Detection System](#5-lane-detection-system)
6. [Navigation & Control](#6-navigation--control)
7. [Configuration Files](#7-configuration-files)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Setup & Installation

### 1.1. Prerequisites

- **OS:** Ubuntu 22.04 LTS
- **ROS2:** Humble
- **Python:** 3.10+
- **Dependencies:** Gazebo, Nav2, SLAM Toolbox

### 1.2. Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  xterm \
  python3-opencv \
  python3-tk \
  python3-pip

pip install customtkinter
```

### 1.3. Building the Project

The repo root **is** the colcon workspace (`build/`, `install/` and `log/` are generated next to `src/`):

```
esda_simulation_2025/          <- workspace root, run everything from here
├── src/
│   ├── esda_simulation_2025/  # this package: URDF, worlds, launch, configs, Python nodes
│   └── esda_hardware_2025/    # ros2_control hardware plugin for the real robot
├── tools/                     # offline tests and world generators (no ROS needed)
├── build/  install/  log/
```

```bash
git clone <repository-url> ~/esda_simulation_2025
cd ~/esda_simulation_2025
colcon build --packages-select esda_simulation_2025
source install/setup.bash
```

> **Build without `--symlink-install`.** This workspace is built in copy mode, so
> Python nodes are *copied* into `install/`. After editing any script, rebuild
> (or press **Colcon Build** in the GUI) and restart the node, or you will still
> be running the old copy. Mixing the two modes fails with
> `failed to create symbolic link ... Is a directory`; if that happens, rebuild with
> `colcon build --packages-select esda_simulation_2025 --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=0`.

**Add to your `~/.bashrc` for convenience:**
```bash
echo "source ~/esda_simulation_2025/install/setup.bash" >> ~/.bashrc
```

---

## 2. Launching the Simulation

### 2.1. 🎮 Using the Simulation Manager GUI (Recommended)

Run it from the workspace root:

```bash
cd ~/esda_simulation_2025
python3 src/esda_simulation_2025/scripts/ui_launch.py
```

Each button opens its own `xterm` window, so `xterm` must be installed.

![Simulation Manager Interface](./assets/ui_screenshot.png)

#### **Buttons**

| Button | What it runs | When to use |
|--------|--------------|-------------|
| **1. Colcon Build** | `colcon build --packages-select esda_simulation_2025` | After changing any code or config |
| **2. Launch Simulation** | `launch_sim.launch.py` (Gazebo, robot, bridges, controllers, EKF). It also rebuilds first | Always first |
| **Launch SLAM** | `online_async_launch.py` | Building a new map. Also used for IGVC runs |
| **Launch AMCL** | `localization_launch.py` | Driving on a saved map |
| **Launch Nav2** | `navigation_launch.py` | Anything autonomous |
| **Launch Lane Detection** | the detector picked in the dropdown | Lane-aware navigation |
| **Launch RViz2** | `rviz2 -d config/view_bot.rviz` | Visualisation |
| **WASD Teleop** | `teleop_wasd.py` | Manual driving |
| **Waypoint Nav** | the *old* fixed-waypoint `waypoint_navigator.py` | Legacy only. For IGVC runs see [section 3](#3-autonomous-igvc-run-waypoint_navigator_recommendationpy) |
| **Follow the Gap / Track Follower / Behaviour Tree** | the reactive stack (bypasses Nav2) | Don't run it alongside Nav2 goals |
| **KILL ALL PROCESSES & RESET GAZEBO** | kills everything and clears stale Gazebo state | When anything gets stuck |

#### **Options**

| Option | Values | Notes |
|--------|--------|-------|
| **World SDF** | `igvc.sdf`, `igvc_campus.sdf`, `igvc_carpark.sdf`, `sim_world_*.sdf` | Any world whose name starts with `igvc` spawns the robot at (11, 0), the track start. See [2.3](#23-worlds) |
| **Costmap YAML** | `[New Costmap]` or a map in `maps/` | `[New Costmap]` means SLAM; a saved map means AMCL |
| **Enable LIDAR** | on / off | |
| **GPS** | on / off | Simulated GPS + `navsat_transform`. Off: no GPS sensor, and the waypoint navigator uses the waypoints' map x/y |
| **ZED Mount** | `front`, `pole_top`, `top_front` | Camera position. `top_front` is the real robot (bracket in front of the Velodyne); pair it with LiDAR Mount `low` |
| **LiDAR Mount** | `pole_top`, `low` | `pole_top` puts the VLP-16 on the 1 m pole |
| **Dual Side Cameras (Test)** | on / off | Adds left and right cameras, each with its own lane detector |
| **Enable Lane Detection** | on / off | Switches Nav2, SLAM and AMCL between `/scan_fused` and `/scan`. Turn it on whenever a lane detector runs |
| **Lane detector** | `Adaptive`, `YOLO`, `Regular`, `FCN`, `TwinLiteNet+` | See [5.2](#52-choosing-a-detector) |
| **Visualization** | On / Off | The OpenCV debug window of the lane detector |

#### **Workflows**

**🗺️ Creating a new map:**
1. World: `igvc.sdf`. Costmap: `[New Costmap]`
2. **Launch Simulation**, then **Launch SLAM**, then **Launch RViz2**, then **WASD Teleop**
3. Drive around, then save the map:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f src/esda_simulation_2025/maps/my_map
   ```

**🧭 Point-and-click navigation on a saved map:**
1. Costmap: your saved map. Tick **Enable Lane Detection**
2. **Launch Simulation**, **Launch AMCL**, **Launch Lane Detection**, **Launch Nav2**, **Launch RViz2**
3. In RViz, set **2D Pose Estimate**, then **2D Goal Pose**

**🏁 Autonomous IGVC run:** see [section 3](#3-autonomous-igvc-run-waypoint_navigator_recommendationpy).

### 2.2. Manual Launch (Command Line)

These are the same commands the GUI runs (`scripts/ui_launch.py` is the reference if they ever differ). Run each one in its own terminal from the workspace root, with `install/setup.bash` sourced.

**Simulation:**
```bash
ros2 launch esda_simulation_2025 launch_sim.launch.py \
  world_file:=$PWD/src/esda_simulation_2025/worlds/igvc.sdf \
  spawn_x:=11.0 spawn_y:=0 \
  use_lidar:=true camera_mount:=front lidar_mount:=pole_top
```
Use `robot_model:=robot_dual_camera.urdf.xacro` to get the side cameras.

**SLAM (new map):**
```bash
ros2 launch esda_simulation_2025 online_async_launch.py \
  use_sim_time:=true scan_topic:=/scan_fused
```

**AMCL (saved map, instead of SLAM):**
```bash
ros2 launch esda_simulation_2025 localization_launch.py \
  use_sim_time:=true \
  map:=$PWD/src/esda_simulation_2025/maps/my_map.yaml \
  amcl_base_frame_id:=base_link amcl_odom_frame_id:=odom \
  scan_topic:=/scan_fused
```

**Lane detection** (pick one; see [5.2](#52-choosing-a-detector)):
```bash
ros2 run esda_simulation_2025 lane_detection_esda.py --ros-args \
  -p detector_method:=adaptive -p show_visualization:=true
```

**Nav2:**
```bash
# With SLAM:
ros2 launch esda_simulation_2025 navigation_launch.py \
  use_sim_time:=true map_subscribe_transient_local:=true scan_topic:=/scan_fused

# With AMCL, add the map:
ros2 launch esda_simulation_2025 navigation_launch.py \
  use_sim_time:=true map_subscribe_transient_local:=true scan_topic:=/scan_fused \
  map:=$PWD/src/esda_simulation_2025/maps/my_map.yaml
```
Use `scan_topic:=/scan` for SLAM, AMCL and Nav2 if no lane detector is running. Nothing publishes `/scan_fused` without one, and Nav2 would then see no obstacles at all.

**RViz2:**
```bash
rviz2 -d src/esda_simulation_2025/config/view_bot.rviz --ros-args -p use_sim_time:=true
```
Set the **Fixed Frame** to `map`.

**Teleop (WASD):**
```bash
python3 src/esda_simulation_2025/scripts/teleop_wasd.py
```
- **W**: Forward | **S**: Backward | **A**: Turn Left | **D**: Turn Right | **X**: Stop
- **U/J**: Increase/Decrease Linear Speed | **I/K**: Increase/Decrease Angular Speed

### 2.3. Worlds

| World | What's around the track | What the lidar sees |
|-------|-------------------------|---------------------|
| `igvc.sdf` | Nothing (bare ground plane) | Most flat-scan rays return `inf`, so the map stays mostly unknown. This is the hardest test |
| `igvc_campus.sdf` | Buildings and a 2 m perimeter wall | Every ray returns, so the map fills in fast |
| `igvc_carpark.sdf` | A parking lot like the competition site: yellow stalls, light poles, parked cars, tents, a picket fence, trees | About 64% of rays return, with real gaps between them |
| `sim_world_*.sdf` | Older test worlds | Spawn at (0, 0) |

All three `igvc*` worlds place the same track at the same pose, so maps and waypoints carry over between them. `igvc_campus.sdf` and `igvc_carpark.sdf` are generated. To change them, edit the layout tables in `tools/generate_campus_world.py` or `tools/generate_carpark_world.py`, re-run the script, and rebuild.

---

## 3. Autonomous IGVC Run (`waypoint_navigator_recommendation.py`)

This is the node that drives the course on its own. Each cycle it creates the next Nav2 goal from the lanes, the lidar and the costmaps. It heads for the GPS waypoints in `waypoints/igvc_waypoints.yaml` when they are in range. It does **not** have a GUI button.

### 3.1. Launch order

1. **GUI:** World `igvc.sdf` (or `igvc_carpark.sdf`). Costmap `[New Costmap]`. Tick **Enable Lane Detection**.
2. **Launch Simulation**, wait for Gazebo, then **Launch SLAM**.
3. **Launch Lane Detection** (`Adaptive` is a good default).
4. **Launch Nav2**, then **Launch RViz2** (optional, but you will want to see it).
5. In a new terminal, **from the workspace root**:
   ```bash
   cd ~/esda_simulation_2025
   source install/setup.bash
   ros2 run esda_simulation_2025 waypoint_navigator_recommendation.py
   ```

> **Run it from the workspace root.** It opens
> `src/esda_simulation_2025/waypoints/igvc_waypoints.yaml` with a *relative*
> path, so it crashes with `FileNotFoundError` from anywhere else.

> **Don't run it alongside another goal source.** That means the **Waypoint Nav**
> button (the old navigator), the behaviour tree, or RViz *2D Goal Pose*. They all
> fight over Nav2.

After editing the script, rebuild (`colcon build --packages-select esda_simulation_2025`) and restart it. The installed copy is what `ros2 run` executes.

### 3.2. What it does

Goals are tried in this order:

| Situation | Goal it sends |
|-----------|---------------|
| Next GPS waypoint within `gps_distance_threshold` (10 m) | The waypoint itself. The waypoint advances when Nav2 reports success, or when the robot passes within 0.5 m of it |
| Lanes visible | A **far goal** 3–6 m down the lane, checked with Nav2 `getPath`. Otherwise a short **lane goal** on the centreline |
| No usable lane goal | A short **forward goal** straight ahead. Once the lanes have ended (seen at least once, then neither line seen for 3 s, like in no-man's-land), it aims at the next GPS waypoint instead, turning at most 60° per goal |
| Nothing sendable for 5 s | **Stuck nudge**: 0.8 m in the clearest nearby direction |
| Nudge finds nothing (dead end) | **Back up** 0.6 m (Nav2 BackUp), then recovery |
| Nav2 goal fails | **Recovery**: a clear spot within 1.5 m ahead. If there isn't one, a **far recovery** goal 3–8 m past the blockage that Nav2 can plan to. If both fail for 20 s, it retries once ignoring unknown space, then releases the lock |

Two guards apply throughout:
- The node remembers the direction of real progress (re-measured every 2 m). Nudge, recovery and forward goals may not point more than 100° away from it, so sidestepping round obstacles can't add up to a U-turn back down the course.
- Places it has already driven are marked in a keepout layer (`/travel_history`, `/keepout_filter_mask`), which discourages revisiting.

### 3.3. GPS waypoints

`src/esda_simulation_2025/waypoints/igvc_waypoints.yaml`. Only the `waypoints:` list is used, in order. Positions are in the **`map` frame, whose origin is the robot's start pose** (as with a fresh SLAM map), so they only line up if the robot starts at the spawn point. The `competition_northbound` / `competition_southbound` lists are approximate real-course positions, kept for reference.

The waypoint file is read when the node starts, so restart the node after editing it. No rebuild is needed.

**GPS.** The `igvc*` worlds have a GPS origin (Oakland University, approximate), and the robot carries a simulated GPS (`description/gps.xacro`, 0.5 m noise) publishing `/gps/fix`. `navsat_transform_node` (started with the EKF, `config/navsat_sim.yaml`) converts it into the odom frame and serves `/fromLL`. Waypoints that have `latitude`/`longitude` are converted once through `/fromLL`. Because the datum pins the start pose, the result is already in the map frame (same origin as the SLAM map). The log shows each conversion and how far it lands from the YAML x/y. The GPS is not fused into SLAM or the EKF; it only places the waypoints. Without GPS (untick **GPS**, or `use_gps:=false` on `launch_sim.launch.py`, other worlds, or `-p use_gps_latlon:=false` on the navigator) it falls back to `position` x/y. `python3 tools/sim_gps.py` prints the latitude/longitude of any sim point and regenerates the datum and the waypoints' coordinates.

### 3.4. Parameters

Pass with `--ros-args -p name:=value`, e.g.:
```bash
ros2 run esda_simulation_2025 waypoint_navigator_recommendation.py --ros-args \
  -p gps_distance_threshold:=15.0 -p enable_far_goals:=false
```

| Parameter | Default | Effect |
|-----------|---------|--------|
| `gps_distance_threshold` | 10.0 | Distance (m) within which the GPS waypoint itself becomes the goal |
| `gps_waypoint_radius_threshold` | 0.5 | Counts a waypoint as reached when this close (m) |
| `gps_retry_delay` | 5.0 | After a failed GPS goal, use normal goals for this long (s) |
| `enable_gps_heading_pull` | true | Aim forward goals at the waypoint once the lanes end |
| `gps_heading_no_lane_timeout` | 3.0 | Seconds without any lane line before the pull starts |
| `gps_heading_max_turn_deg` | 60 | Largest turn toward the waypoint per goal |
| `enable_far_goals` | true | Goals 3–6 m down the lane |
| `far_goal_max_detour` | 1.6 | Reject a far goal whose path is longer than this × straight-line distance |
| `unknown_clearance_allowance` | 0.8 | How far into unknown space a goal may sit (m). Interacts with the recovery timeout; change both together |
| `enable_travel_keepout` | true | Discourage revisiting driven areas |
| `replan_while_moving` | true | Let a better goal replace the active one |

Some recovery settings are plain attributes in `__init__`, not ROS parameters: dead-end backup distance and speed, far-recovery distances and detour, and the progress-heading limits.

### 3.5. Reading its log

| Log line | Meaning |
|----------|---------|
| `GPS waypoint N is too far: … Using normal navigation.` | Waypoint out of range, so it is following the lanes |
| `Not enough left/right lane points.` | Fewer than two points on one side. No centreline, so no lane or far goals |
| `FAR GOAL \| …` | A far-goal decision (`none`, `rejected, no path`, `dropped, detour`, …) |
| `Stuck nudge: …` / `Dead end: backing up` | The stuck ladder |
| `FAR RECOVERY \| …` | Whether it found a route past a blockage |
| `Recovery has produced nothing for 20 s` | Gave up on recovery and is resuming normal goals |

### 3.6. Offline tests

The waypoint logic has a test harness that needs no ROS, Gazebo or Nav2. Run it after changing the navigator or `nav2_params.yaml`:
```bash
python3 tools/validate_waypoint_logic.py
```

---

## 4. System Architecture

### 4.1. Overall System Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    ESDA Simulation Manager (UI)                  │
│                      ui_launch.py (Tkinter GUI)                  │
└────────────┬─────────────────────────────────────────┬──────────┘
             │                                         │
             v                                         v
┌────────────────────────┐                  ┌──────────────────────┐
│   Gazebo Simulation    │                  │    ROS2 Nodes        │
│  - Physics Engine      │                  │  - robot_state_pub   │
│  - World Environment   │                  │  - joint_state_pub   │
│  - Robot Model         │◄─────────────────┤  - gz_bridge         │
│  - Sensors (LiDAR,Cam) │                  │  - tf2               │
└────────────┬───────────┘                  └──────────┬───────────┘
             │                                         │
             v                                         v
┌────────────────────────────────────────────────────────────────┐
│                       Sensor Topics                            │
│  /scan (LaserScan) → LiDAR data                               │
│  /camera/left/image_raw → Left camera                         │
│  /camera/right/image_raw → Right camera                       │
│  /camera/depth/image_raw → Depth image                        │
│  /odom → Odometry                                              │
└────────────┬──────────────────────────────┬──────────────────┘
             │                              │
             v                              v
┌────────────────────────┐       ┌──────────────────────────────┐
│  Lane Detection Node   │       │   Navigation Stack (Nav2)    │
│  lane_detection.py     │       │                              │
│  - CV Lane Detection   │       │  Components:                 │
│  - 3D Point Projection │       │  1. AMCL (Localization)      │
│  - Scan Fusion         │       │  2. Planner (A* pathfinding) │
│  Publishes:            │───────►  3. Controller (MPPI)        │
│  - /lane_obstacles     │       │  4. Costmap Layers           │
│  - /lane_markers       │       │  5. Recovery Behaviors       │
│  - /scan_fused         │       │                              │
└────────────────────────┘       └──────────┬───────────────────┘
                                            │
                                            v
                                 ┌──────────────────────┐
                                 │   /cmd_vel           │
                                 │   Robot Control      │
                                 └──────────────────────┘
```

### 4.2. Key Topics & Data Flow

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/scan` | LaserScan | Gazebo | SLAM, Costmap | Raw LiDAR data |
| `/scan_fused` | LaserScan | lane_detection | Costmap, AMCL | LiDAR + lane obstacles |
| `/lane_obstacles` | PointCloud2 | lane_detection | Costmap | 3D lane points |
| `/lane_markers` | MarkerArray | lane_detection | RViz | Visualization |
| `/odom` | Odometry | Gazebo | Nav2, SLAM | Robot position |
| `/cmd_vel` | Twist | Nav2/Teleop | Gazebo | Robot commands |
| `/map` | OccupancyGrid | SLAM/Map Server | AMCL, Planner | Environment map |
| `/camera/*/image_raw` | Image | Gazebo | lane_detection | Camera feeds |

### 4.3. Transform (TF) Tree

```
map
 └─ odom
     └─ base_link
         ├─ base_footprint
         ├─ laser_frame (LiDAR)
         ├─ camera_link
         │   └─ camera_link_optical
         ├─ left_wheel_link
         ├─ right_wheel_link
         └─ caster_wheel_link
```

- **map → odom**: Published by AMCL (localization) or SLAM
- **odom → base_link**: Published by Gazebo (odometry)
- **base_link → sensors**: Published by robot_state_publisher (from URDF)

---

## 5. Lane Detection System

### 5.1. Overview

The lane detection system uses computer vision to detect white lane markings and publishes them as obstacles for Nav2's costmap, enabling lane-aware autonomous navigation.

### 5.2. Choosing a Detector

All five detectors share the base class in `lane_detection.py`. They publish the same `/lane_markers`, `/lane_obstacles` and `/scan_fused`, so Nav2 and the waypoint navigator don't care which one runs. They differ only in how the white-paint mask is found.

| Dropdown | Script | Mask | Needs |
|----------|--------|------|-------|
| **Adaptive** | `lane_detection_esda.py`, `detector_method:=adaptive` | Local (neighbourhood) threshold. Throws away any mask covering more than 15% of the image region | Nothing extra |
| **YOLO** | `lane_detection_esda.py`, `detector_method:=yolo` | YOLOv8-seg `lane` class, with the model's `cone` pixels (grown by 15 px) cut out, so barrel stripes aren't read as lane | The `~/esda_venvs/yolo` venv and weights at `~/esda_models/lane_seg.pt` (see [5.3](#53-yolo-model-setup-training-and-viewing)). Falls back to Adaptive if either is missing |
| **Regular** | `lane_detection.py` | Fixed grayscale threshold of 130 | Nothing extra |
| **FCN** | `lane_detection_FCN.py` | FCN segmentation | `lane-detection-on-rural-roads-master/.../FCN_model.h5` in the workspace root |
| **TwinLiteNet+** | `lane_detection_twinlite.py` | TwinLiteNet+ (nano) | `TwinLiteNetPlus/` repo and weights in the workspace root (see `LANE_DETECTION.md`) |

**Adaptive vs Regular:** the yolo-adaptive branch benchmarked them on 395 frames. Under a shadow band, the fixed threshold's recall fell from 0.800 to 0.678; under a lighting gradient, its precision fell from 0.452 to 0.178. Adaptive held 0.79 recall in every lighting condition.

**Cones are obstacles** in both Adaptive and YOLO mode. Adaptive finds them with the base class's orange HSV thresholds; YOLO uses the model's `cone` class. Cone pixels are back-projected with the depth image and injected into `/scan_fused` next to the lane points, so the costmaps, SLAM and AMCL see them even where the pole-mounted lidar ring passes over a low cone. They are also cut out of the lane mask, and published on `/cone_obstacles` (PointCloud2) for RViz. Turn this off with `-p cone_obstacles:=false`.

`lane_detection_esda.py` has its own self-test, which needs no ROS:
```bash
python3 src/esda_simulation_2025/scripts/lane_detection_esda.py --selftest
```

Useful parameters (`--ros-args -p …`): `detector_method`, `max_area_frac` (0.15), `adaptive_block` (61), `adaptive_c` (-8), `yolo_weights`, `yolo_conf` (0.25), `yolo_cone_dilate_px` (15), `yolo_device` (e.g. `0`), `yolo_image_topic`, `cone_obstacles` (true), `cone_max_range` (8.0 m), `cone_min_area_px` (150).

### 5.3. YOLO Model: Setup, Training and Viewing

**Environment.** YOLO runs from its own venv with a CUDA torch and `ultralytics`. The venv still sees the system ROS packages, and it leaves the system torch (used by TwinLiteNet+) alone:
```bash
python3 -m venv --system-site-packages ~/esda_venvs/yolo
~/esda_venvs/yolo/bin/pip install torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cu121
~/esda_venvs/yolo/bin/pip install ultralytics==8.3.40 "numpy<2"
```
`numpy<2` keeps `cv_bridge` working. The GUI's **YOLO** option runs the detector with this venv's Python automatically. By hand:
```bash
ros2 run --prefix ~/esda_venvs/yolo/bin/python3 esda_simulation_2025 lane_detection_esda.py \
  --ros-args -p detector_method:=yolo -p yolo_weights:=$HOME/esda_models/lane_seg.pt
```

**Watching it.** The annotated view (tinted masks, a box and confidence for every `lane` and `cone`) is on `/lane_detection/yolo_image`. It is only drawn while something subscribes:
```bash
ros2 run rqt_image_view rqt_image_view /lane_detection/yolo_image
```

**Training.** Weights are gitignored, so train them on the GPU with the tools in `research/vision/`. Start from the stock COCO `yolov8n-seg.pt` and fine-tune on two kinds of data:
1. **Synthetic frames** from `synth_gen.py`, rendered for the sim camera. Pick the mount with `ESDA_VISION_CONFIG`; `camera_front.yaml` and `camera_pole_top.yaml` are derived from the URDF:
   ```bash
   cd research/vision
   TEX=../../src/esda_simulation_2025/worlds/models/orange_igvc/materials/textures
   ESDA_VISION_CONFIG=camera_front.yaml    python3 synth_gen.py --textures $TEX --out ~/esda_datasets/lane_seg_front    --n-train 2000 --n-val 300
   ESDA_VISION_CONFIG=camera_pole_top.yaml python3 synth_gen.py --textures $TEX --out ~/esda_datasets/lane_seg_pole_top --n-train 2000 --n-val 300
   ```
2. **Real Gazebo frames** from `gz_autolabel.py`. It renders RGB and depth from cameras at known poses on the track. A pixel is labelled `lane` where its ground point lands on the track's paint texture, and `cone` where its 3D point falls inside a barrel. Shadows are in the images but not in the labels:
   ```bash
   python3 gz_autolabel.py render --world igvc.sdf --mount front --n 30 --seed 1 --out ~/esda_capture/gz_igvc
   # ...repeat for other seeds, mounts, and igvc_carpark.sdf...
   python3 gz_autolabel.py label --capture ~/esda_capture/gz_igvc --out ~/esda_datasets/lane_seg_gz --overlay
   ```
   Check `~/esda_datasets/lane_seg_gz/overlay/` by eye before training.

Then train with `ultralytics` from the venv, pointing `data=` at a yaml that lists both datasets (see `~/esda_datasets/lane_seg_mixed.yaml`), and copy `runs/.../weights/best.pt` to `~/esda_models/lane_seg.pt`.

### 5.4. Lane Detection Pipeline

```
Camera Images → CV Processing → 3D Projection → Scan Fusion → Costmap
```

**Detailed Steps:**

1. **Image Acquisition**
   - Subscribe to `/camera/left/image_raw`, `/camera/right/image_raw`, `/camera/depth/image_raw`
   - 640x480 resolution, ~30 Hz

2. **Computer Vision Processing** (`detect_white_lanes`)
   - Convert to grayscale
   - Apply ROI mask (lower 50% of image)
   - Threshold for white pixels (130-255)
   - Morphological operations (noise removal)
   - Canny edge detection
   - Hough Line Transform (detect line segments)
   - Profile validation (ensure Dark-White-Dark pattern)

3. **3D Point Projection** (`publish_obstacle_cloud`)
   - For each detected line segment:
     - Sample points densely (every 2 pixels)
     - Sample across line thickness (8 pixels wide)
     - Use depth image to get Z coordinate
     - Fallback to ground plane projection if depth invalid
   - Result: 100-120 3D points per lane segment

4. **Scan Fusion** (`scan_callback`)
   - Transform 3D points from `camera_link_optical` to `laser_frame`
   - Convert to polar coordinates (range, angle)
   - Inject into LaserScan message
   - Publish fused scan on `/scan_fused`

5. **Costmap Integration**
   - Nav2's VoxelLayer/ObstacleLayer subscribes to:
     - `/scan_fused` (LaserScan with lanes)
     - `/lane_obstacles` (PointCloud2 backup)
   - Lane points marked as obstacles
   - Inflation layer adds safety buffer

### 5.5. Lane Detection Parameters

Located in `scripts/lane_detection.py`:

```python
# Thresholds
white_threshold_low: 130          # Grayscale threshold for white detection
white_threshold_high: 255

# Line Detection
min_line_length: 30               # Minimum pixels for valid line
max_line_gap: 40                  # Max gap in pixels to connect lines

# Dense Sampling
lane_thickness_pixels: 8          # Width of lane in pixels
point_spacing_pixels: 2.0         # Distance between sampled points

# Camera Model
fx = 640 / (2 * tan(1.089 / 2))  # Focal length (from FOV)
camera_height = 0.315             # Height above ground (meters)
```

### 5.6. Visualization

**In RViz:**
- **Purple/Magenta LaserScan points** (`/scan_fused`) - Fused LiDAR + lanes
- **Purple PointCloud** (`/lane_obstacles`) - Raw 3D lane points  
- **Yellow cube markers** (`/lane_markers`) - Lane segment positions
- **LocalCostmap** - Shows lanes as lethal obstacles (black) with inflation (cyan)

**In OpenCV Window** (if `show_visualization: true`):
- Top row: Stereo camera feeds (left & right)
- Bottom row: White mask & edge detection
- Green lines overlaid on detected lanes

### 5.7. Toggling Lane Detection

**Via UI:**
- Check/uncheck **"Enable Lane Detection"** checkbox
- Automatically switches between `/scan` and `/scan_fused`

**Manual:**
```bash
# Without lanes:
scan_topic:=/scan

# With lanes:
scan_topic:=/scan_fused
```

---

## 6. Navigation & Control

### 6.1. Nav2 Stack Overview

The robot uses Nav2 (Navigation2) with the following configuration:

**Components:**
- **AMCL**: Adaptive Monte Carlo Localization for robot positioning
- **Planner**: A* algorithm for global path planning (NavfnPlanner)
- **Controller**: MPPI (Model Predictive Path Integral) for trajectory following
- **Recovery Behaviors**: Spin, backup, wait for obstacle avoidance
- **Costmap layers**: Static map, obstacles (LiDAR/lanes), inflation

### 6.2. Optimized Parameters

The system is tuned for stable performance:

| Component | Key Settings | Purpose |
|-----------|--------------|---------|
| **Controller** | 8 Hz frequency | Stable control loop |
| **MPPI** | 1200 batch size, 40 timesteps | Reduced computation (57%) |
| **Inflation** | 0.28m radius, 3.5 cost scaling | Moderate safety margins |
| **Planner** | A* algorithm, 0.5m tolerance | Fast, accurate paths |
| **Costmaps** | 0.05m resolution, 3 Hz updates | Balance detail/speed |

### 6.3. Navigation Capabilities

**Minimum Navigable Gap:**
- Robot radius: 0.55m
- Inflation: 0.28m per side
- **Total: 1.11m** minimum gap width

**Performance Metrics:**
- Control frequency: **8 Hz** (125ms period)
- Planning frequency: **1 Hz**
- Maximum velocity: **0.5 m/s**
- Obstacle detection range: **4.5m**

### 6.4. Costmap Layers

**Local Costmap** (6m × 6m, rolling window):
1. **VoxelLayer**: 3D obstacle representation from sensors
   - Observation sources: `/scan_fused`, `/lane_obstacles`
2. **InflationLayer**: 28cm safety buffer around obstacles

**Global Costmap** (50m × 50m, static):
1. **StaticLayer**: Pre-mapped obstacles from SLAM
2. **ObstacleLayer**: Dynamic obstacles from sensors
3. **InflationLayer**: 28cm safety buffer

### 6.5. Setting Navigation Goals

**In RViz:**
1. Click **"2D Goal Pose"** tool
2. Click and drag on map to set position and orientation
3. Robot automatically plans and executes path

**Expected Behavior:**
- Robot drives **forward immediately** (no spinning)
- Follows smooth path avoiding obstacles and lanes
- Respects lane boundaries as hard constraints
- Adjusts path dynamically for moving obstacles

---

## 7. Configuration Files

### 7.1. Key Files

| File | Purpose | Key Parameters |
|------|---------|----------------|
| `config/nav2_params.yaml` | Nav2 configuration | Controller freq, MPPI settings, inflation |
| `config/view_bot.rviz` | RViz visualization | Display configs, colors, topics |
| `config/mapper_params_online_async.yaml` | SLAM Toolbox settings | Mapping parameters |
| `config/mapper_params_localization.yaml` | AMCL settings | Localization parameters |
| `description/robot_core_ref.xacro` | Robot URDF | Geometry, sensors, physics |
| `worlds/*.sdf` | Gazebo worlds | Environment definitions |

### 7.2. Important Nav2 Settings

**Controller (MPPI):**
```yaml
controller_frequency: 8.0           # Control loop rate
batch_size: 1200                    # Trajectory samples
time_steps: 40                      # Prediction horizon
model_dt: 0.125                     # Timestep (matches frequency)
```

**Costmaps:**
```yaml
resolution: 0.05                    # 5cm grid cells
update_frequency: 3.0               # Local costmap updates/sec
inflation_radius: 0.28              # Safety buffer (meters)
cost_scaling_factor: 3.5            # Cost gradient steepness
```

**Planner:**
```yaml
plugin: "nav2_navfn_planner/NavfnPlanner"
use_astar: true                     # A* algorithm (not Dijkstra)
tolerance: 0.5                      # Goal tolerance (meters)
```

### 7.3. Modifying Parameters

**After changing configs or scripts:**
```bash
cd ~/esda_simulation_2025
colcon build --packages-select esda_simulation_2025
source install/setup.bash
```
Then restart the affected nodes.

Or use **"Colcon Build"** button in UI.

---

## 8. Troubleshooting

### 8.1. Common Issues

**❌ "Failed to create a plan" errors**
- **Cause**: Goal in obstacle or unreachable
- **Fix**: Set goals in free space, check costmap in RViz

**❌ "Control loop missed desired rate" warnings**
- **Cause**: CPU overload
- **Fix**: Reduce `batch_size` further (1000, 800)

**❌ Robot spins instead of driving forward**
- **Cause**: Planner failing (should be fixed now)
- **Fix**: Check `use_astar: true` in nav2_params.yaml

**❌ Lane detection not appearing**
- **Cause**: No white lanes in world, or checkbox unchecked
- **Fix**: Use lanes-enabled world, check **"Enable Lane Detection"**

**❌ Purple obstacles not showing in RViz**
- **Cause**: Display not enabled
- **Fix**: Enable **"LaserScan_Fused"** and **"LaneObstacles"** in RViz

**❌ `failed to load joint_state_broadcaster` / robot doesn't spawn**
- **Cause**: Stale Gazebo or DDS state from a previous run
- **Fix**: **KILL ALL PROCESSES & RESET GAZEBO** in the GUI, or:
  ```bash
  pkill -f ign; pkill -f gz; rm -f /dev/shm/fastrtps_*
  ```

**❌ `waypoint_navigator_recommendation.py` crashes with `FileNotFoundError`**
- **Cause**: Started outside the workspace root (the waypoint file path is relative)
- **Fix**: `cd ~/esda_simulation_2025` first. See [section 3.1](#31-launch-order)

**❌ Code changes have no effect**
- **Cause**: `ros2 run` executes the copy in `install/`, not the file in `src/`
- **Fix**: Rebuild (`colcon build --packages-select esda_simulation_2025`) and restart the node

**❌ Gazebo crashes or freezes**
- **Cause**: GPU issues or resource limits
- **Fix**: 
  ```bash
  export LIBGL_ALWAYS_SOFTWARE=1  # Software rendering
  killall gzserver gzclient        # Kill hung processes
  ```

### 8.2. Verification Commands

**Check active topics:**
```bash
ros2 topic list
ros2 topic hz /scan_fused          # Should show ~30 Hz
ros2 topic hz /cmd_vel             # Should show ~8 Hz when navigating
```

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

**Monitor Nav2 status:**
```bash
ros2 topic echo /controller_server/transition_event
ros2 topic echo /planner_server/transition_event
```

**Check lane detection:**
```bash
ros2 topic echo /lane_obstacles --once
ros2 topic echo /lane_markers --once
```

### 8.3. Performance Tuning

**If navigation too conservative:**
```yaml
inflation_radius: 0.25              # Reduce inflation
cost_scaling_factor: 3.0            # Gentler cost gradient
```

**If navigation too aggressive:**
```yaml
inflation_radius: 0.35              # Increase inflation
cost_scaling_factor: 5.0            # Steeper cost gradient
failure_tolerance: 0.3              # Stricter tolerance
```

**If controller still missing rate:**
```yaml
controller_frequency: 7.0           # Lower frequency
batch_size: 1000                    # Fewer samples
time_steps: 35                      # Shorter horizon
```

---

## 9. Additional Resources

### 9.1. Documentation

- **Lane Detection Details**: `LANE_DETECTION_FIXES.md`
- **Nav2 Official Docs**: https://navigation.ros.org/
- **SLAM Toolbox**: https://github.com/SteveMacenski/slam_toolbox
- **ROS2 Humble**: https://docs.ros.org/en/humble/

### 9.2. Useful Commands

**Save map:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

**View robot model:**
```bash
ros2 launch esda_simulation_2025 rsp.launch.py
rviz2  # Add RobotModel display
```

**Record data:**
```bash
ros2 bag record -a  # Record all topics
ros2 bag play <bag_file>  # Playback
```

### 9.3. File Structure Reference

```
esda_simulation_2025/
├── config/
│   ├── nav2_params.yaml              # ★ Nav2 configuration
│   ├── view_bot.rviz                 # ★ RViz config
│   ├── mapper_params_*.yaml          # SLAM configs
│   └── *.yaml                        # Other configs
├── description/
│   ├── robot_core_ref.xacro          # ★ Robot definition
│   └── *.xacro                       # Component xacros
├── launch/
│   ├── launch_sim.launch.py          # ★ Main simulation
│   ├── navigation_launch.py          # ★ Nav2 launcher
│   ├── online_async_launch.py        # SLAM launcher
│   └── *.launch.py                   # Other launchers
├── scripts/
│   ├── ui_launch.py                  # ★ GUI manager
│   ├── waypoint_navigator_recommendation.py  # ★ Autonomous IGVC navigator
│   ├── read_gps_waypoints.py         # Loads waypoints/igvc_waypoints.yaml
│   ├── lane_detection.py             # ★ Lane detection base class (Regular)
│   ├── lane_detection_esda.py        # Adaptive / YOLO detector
│   ├── lane_detection_FCN.py         # FCN detector
│   ├── lane_detection_twinlite.py    # TwinLiteNet+ detector
│   └── teleop_wasd.py                # Keyboard control
├── waypoints/
│   └── igvc_waypoints.yaml           # ★ GPS waypoints (map frame)
├── worlds/
│   ├── igvc.sdf, igvc_campus.sdf, igvc_carpark.sdf
│   └── models/                       # orange_igvc track, esda_campus, esda_carpark
├── maps/
│   └── *.yaml, *.pgm                 # Saved costmaps
└── README.md                         # This file

tools/ (workspace root)
├── validate_waypoint_logic.py        # Offline tests for the navigator
├── generate_campus_world.py          # Writes worlds/models/esda_campus
└── generate_carpark_world.py         # Writes worlds/models/esda_carpark
```

**★ = Most frequently modified files**

---

## 10. Credits & License

**Developed by:** ESDA Team 2025  
**License:** [Specify License]  
**ROS2 Version:** Humble  
**Last Updated:** October 2026

For questions or issues, please open an issue on the repository.