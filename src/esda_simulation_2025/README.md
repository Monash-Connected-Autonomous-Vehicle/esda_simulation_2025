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

---

## 📋 Table of Contents

1. [Setup & Installation](#1-setup--installation)
2. [Launching the Simulation](#2-launching-the-simulation)
3. [System Architecture](#3-system-architecture)
4. [Lane Detection System](#4-lane-detection-system)
5. [Navigation & Control](#5-navigation--control)
6. [Configuration Files](#6-configuration-files)
7. [Troubleshooting](#7-troubleshooting)

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
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  python3-opencv \
  python3-tk
```

### 1.3. Building the Project

**Clone the repository into your workspace:**
```bash
mkdir -p ~/esda_sim_ws/src
cd ~/esda_sim_ws/src
git clone <repository-url> esda_simulation_2025
```

**Your folder structure should look like:**
```
esda_sim_ws/
├── src/
│   └── esda_simulation_2025/
│       ├── config/          # Nav2, SLAM, RViz configs
│       ├── description/     # Robot URDF/Xacro files
│       ├── launch/          # Launch files
│       ├── maps/            # Saved costmaps
│       ├── scripts/         # Python scripts & UI
│       ├── worlds/          # Gazebo world files
│       └── CMakeLists.txt
├── build/
├── install/
└── log/
```

**Build the project:**
```bash
cd ~/esda_sim_ws
colcon build --symlink-install
source install/setup.bash
```

**Add to your `~/.bashrc` for convenience:**
```bash
echo "source ~/esda_sim_ws/install/setup.bash" >> ~/.bashrc
```

---

## 2. Launching the Simulation

### 2.1. 🎮 Using the Simulation Manager GUI (Recommended)

The **ESDA Simulation Manager** provides a complete graphical interface:

```bash
python3 ~/esda_sim_ws/src/esda_simulation_2025/scripts/ui_launch.py
```

![Simulation Manager Interface](./assets/ui_screenshot.png)

#### **GUI Components:**

| Button | Function | When to Use |
|--------|----------|-------------|
| **Colcon Build** | Compiles code changes | After modifying code |
| **Launch Simulation** | Starts Gazebo + Robot | Always first |
| **Launch SLAM** | Starts mapping mode | Creating new maps |
| **Launch AMCL** | Starts localization | Using existing maps |
| **Launch Nav2** | Enables autonomous navigation | For autonomous driving |
| **Launch RViz2** | Opens visualization | To see robot/map/sensors |
| **Launch WASD Teleop** | Keyboard control | Manual driving |
| **Kill All Processes** | Emergency stop | To reset everything |

#### **Dropdowns:**

- **Select World:** Choose `.sdf` Gazebo world file
- **Select Costmap:** 
  - `[New Costmap]` → SLAM mapping mode
  - Select existing map → AMCL localization mode
- **☑️ Enable Lane Detection:** Toggles camera-based lane obstacle detection

#### **Workflow Examples:**

**🗺️ Creating a New Map:**
1. Select World: `competition_course.sdf`
2. Select Costmap: `[New Costmap]`
3. ☑️ Uncheck "Enable Lane Detection"
4. Click: **Launch Simulation** → **Launch SLAM** → **Launch RViz2** → **Launch WASD Teleop**
5. Drive around with WASD keys to map the environment
6. Save map: `ros2 run nav2_map_server map_saver_cli -f ~/my_map`

**🧭 Autonomous Navigation:**
1. Select World: `competition_course.sdf`
2. Select Costmap: `my_map.yaml` (your saved map)
3. ☑️ Check "Enable Lane Detection" (for lane-aware navigation)
4. Click: **Launch Simulation** → **Launch AMCL** → **Launch Nav2** → **Launch RViz2**
5. Set initial pose (2D Pose Estimate in RViz)
6. Set navigation goal (2D Goal Pose in RViz)
7. Robot autonomously navigates while respecting lane boundaries

**🛣️ Lane Detection Testing:**
1. Select World: `lanes_course.sdf` (world with white lane markings)
2. ☑️ Check "Enable Lane Detection"
3. Click: **Launch Simulation** → **Launch RViz2**
4. In RViz, observe:
   - **Purple/magenta obstacles** = Detected lane markings
   - **Yellow cube markers** = Lane segment positions
   - **LocalCostmap** = Integration with navigation

### 2.2. Manual Launch (Advanced)

For users preferring command-line control:

**Launch Simulation:**
```bash
ros2 launch esda_simulation_2025 launch_sim.launch.py world:=competition_course.sdf
```

**SLAM Mapping:**
```bash
ros2 launch esda_simulation_2025 online_async_launch.py use_sim_time:=true
```

**AMCL Localization:**
```bash
ros2 launch esda_simulation_2025 localization_launch.py \
  use_sim_time:=true \
  map:=/path/to/map.yaml
```

**Nav2 Navigation:**
```bash
# Without lane detection:
ros2 launch esda_simulation_2025 navigation_launch.py \
  use_sim_time:=true \
  map_subscribe_transient_local:=true \
  map:=/path/to/map.yaml \
  scan_topic:=/scan

# With lane detection:
ros2 launch esda_simulation_2025 navigation_launch.py \
  use_sim_time:=true \
  map_subscribe_transient_local:=true \
  map:=/path/to/map.yaml \
  scan_topic:=/scan_fused
```

**RViz2:**
```bash
rviz2 -d src/esda_simulation_2025/config/view_bot.rviz
```

**Teleop (WASD Control):**
```bash
python3 src/esda_simulation_2025/scripts/teleop_wasd.py
```
- **W**: Forward | **S**: Backward | **A**: Turn Left | **D**: Turn Right | **X**: Stop
- **U/J**: Increase/Decrease Linear Speed | **I/K**: Increase/Decrease Angular Speed

---

## 3. System Architecture

### 3.1. Overall System Diagram

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

### 3.2. Key Topics & Data Flow

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

### 3.3. Transform (TF) Tree

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

## 4. Lane Detection System

### 4.1. Overview

The lane detection system uses computer vision to detect white lane markings and publishes them as obstacles for Nav2's costmap, enabling lane-aware autonomous navigation.

### 4.2. Lane Detection Pipeline

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

### 4.3. Lane Detection Parameters

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

### 4.4. Visualization

**In RViz:**
- **Purple/Magenta LaserScan points** (`/scan_fused`) - Fused LiDAR + lanes
- **Purple PointCloud** (`/lane_obstacles`) - Raw 3D lane points  
- **Yellow cube markers** (`/lane_markers`) - Lane segment positions
- **LocalCostmap** - Shows lanes as lethal obstacles (black) with inflation (cyan)

**In OpenCV Window** (if `show_visualization: true`):
- Top row: Stereo camera feeds (left & right)
- Bottom row: White mask & edge detection
- Green lines overlaid on detected lanes

### 4.5. Toggling Lane Detection

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

## 5. Navigation & Control

### 5.1. Nav2 Stack Overview

The robot uses Nav2 (Navigation2) with the following configuration:

**Components:**
- **AMCL**: Adaptive Monte Carlo Localization for robot positioning
- **Planner**: A* algorithm for global path planning (NavfnPlanner)
- **Controller**: MPPI (Model Predictive Path Integral) for trajectory following
- **Recovery Behaviors**: Spin, backup, wait for obstacle avoidance
- **Costmap layers**: Static map, obstacles (LiDAR/lanes), inflation

### 5.2. Optimized Parameters

The system is tuned for stable performance:

| Component | Key Settings | Purpose |
|-----------|--------------|---------|
| **Controller** | 8 Hz frequency | Stable control loop |
| **MPPI** | 1200 batch size, 40 timesteps | Reduced computation (57%) |
| **Inflation** | 0.28m radius, 3.5 cost scaling | Moderate safety margins |
| **Planner** | A* algorithm, 0.5m tolerance | Fast, accurate paths |
| **Costmaps** | 0.05m resolution, 3 Hz updates | Balance detail/speed |

### 5.3. Navigation Capabilities

**Minimum Navigable Gap:**
- Robot radius: 0.55m
- Inflation: 0.28m per side
- **Total: 1.11m** minimum gap width

**Performance Metrics:**
- Control frequency: **8 Hz** (125ms period)
- Planning frequency: **1 Hz**
- Maximum velocity: **0.5 m/s**
- Obstacle detection range: **4.5m**

### 5.4. Costmap Layers

**Local Costmap** (6m × 6m, rolling window):
1. **VoxelLayer**: 3D obstacle representation from sensors
   - Observation sources: `/scan_fused`, `/lane_obstacles`
2. **InflationLayer**: 28cm safety buffer around obstacles

**Global Costmap** (50m × 50m, static):
1. **StaticLayer**: Pre-mapped obstacles from SLAM
2. **ObstacleLayer**: Dynamic obstacles from sensors
3. **InflationLayer**: 28cm safety buffer

### 5.5. Setting Navigation Goals

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

## 6. Configuration Files

### 6.1. Key Files

| File | Purpose | Key Parameters |
|------|---------|----------------|
| `config/nav2_params.yaml` | Nav2 configuration | Controller freq, MPPI settings, inflation |
| `config/view_bot.rviz` | RViz visualization | Display configs, colors, topics |
| `config/mapper_params_online_async.yaml` | SLAM Toolbox settings | Mapping parameters |
| `config/mapper_params_localization.yaml` | AMCL settings | Localization parameters |
| `description/robot_core_ref.xacro` | Robot URDF | Geometry, sensors, physics |
| `worlds/*.sdf` | Gazebo worlds | Environment definitions |

### 6.2. Important Nav2 Settings

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

### 6.3. Modifying Parameters

**After changing configs:**
```bash
cd ~/esda_sim_ws
colcon build --symlink-install
source install/setup.bash
```

Or use **"Colcon Build"** button in UI.

---

## 7. Troubleshooting

### 7.1. Common Issues

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

**❌ Gazebo crashes or freezes**
- **Cause**: GPU issues or resource limits
- **Fix**: 
  ```bash
  export LIBGL_ALWAYS_SOFTWARE=1  # Software rendering
  killall gzserver gzclient        # Kill hung processes
  ```

### 7.2. Verification Commands

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

### 7.3. Performance Tuning

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

## 8. Additional Resources

### 8.1. Documentation

- **Lane Detection Details**: `LANE_DETECTION_FIXES.md`
- **Nav2 Official Docs**: https://navigation.ros.org/
- **SLAM Toolbox**: https://github.com/SteveMacenski/slam_toolbox
- **ROS2 Humble**: https://docs.ros.org/en/humble/

### 8.2. Useful Commands

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

### 8.3. File Structure Reference

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
│   ├── lane_detection.py             # ★ Lane detection
│   └── teleop_wasd.py                # Keyboard control
├── worlds/
│   └── *.sdf                         # Gazebo worlds
├── maps/
│   └── *.yaml, *.pgm                 # Saved costmaps
└── README.md                         # This file
```

**★ = Most frequently modified files**

---

## 9. Credits & License

**Developed by:** ESDA Team 2025  
**License:** [Specify License]  
**ROS2 Version:** Humble  
**Last Updated:** February 2026

For questions or issues, please open an issue on the repository.