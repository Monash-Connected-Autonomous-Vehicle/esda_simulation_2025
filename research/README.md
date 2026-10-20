# ESDA 2025 — Project Overview & Status

This folder holds the **ESDA autonomous vehicle** work, across two tracks:

| Track | Where | What it is |
|-------|-------|------------|
| **Simulation** | `esda_simulation_2025/` (this folder) | Gazebo sim: ros2_control 2-wheel diff-drive + slam_toolbox + Nav2 + AMCL. The proven software stack. |
| **Hardware (Achilles)** | `../esda-achilles-team-1/` | The physical robot: Raspberry Pi 4 + ESP32 + 2 motors/encoders + RPLidar + ZED2i camera. |

The goal is to take the **proven simulation stack** (SLAM + Nav2 working in Gazebo)
and run it on the **real Achilles robot**. We paused the hardware push to document
progress — see below and `ACHILLES_HARDWARE_PROGRESS.md`.

---

## Current status (one glance)

**Working on hardware:**
- ESP32 motor control with PID + encoders, spin-in-place turns, relay/E-stop.
- Raspberry Pi ↔ ESP32 serial link (GPIO UART) with telemetry.
- Keyboard teleop (`keyboard_drive.py`) + terminal E-stop (`estop.py`).
- RPLidar publishing `/scan`, viewable live in Foxglove.
- **Reactive lidar auto-drive** (`lidar_autodrive.py`): drives forward, stops/turns at obstacles — first real autonomy.

**Not yet done (the sim → hardware gap):**
- A ROS→serial **bridge** connecting Nav2's `cmd_vel` to the ESP32.
- **Wheel odometry** published for EKF/SLAM.
- Running **SLAM + Nav2 on the real robot** (works in sim only).
- Real sensor drivers replacing Gazebo sensors; `use_sim_time=false` everywhere.

Full detail, gotchas, and next steps are in **`ACHILLES_HARDWARE_PROGRESS.md`**.

---

## Running the simulation (this repo)

Clone into a ROS 2 workspace `src/`, build from the workspace root, source, then:

```bash
# Simulator
ros2 launch esda_simulation_2025 launch_sim.launch.py

# Teleop (note the cmd_vel remap)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped

# SLAM (mapping)
ros2 launch slam_toolbox online_async_launch.py

# Localization + Nav2
ros2 launch esda_simulation_2025 localization_launch.py
ros2 launch esda_simulation_2025 navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```

**Key interface** (both sim and hardware must match):
- Movement command: `/diff_drive_base_controller/cmd_vel_unstamped` (`geometry_msgs/Twist`)
- Base frame: **`base_link`** — verified against `description/robot_core_ref.xacro`
  (the URDF root link) and `config/nav2_params.yaml` (`robot_base_frame: base_link`
  in every costmap, AMCL and behaviour-server block). This line previously said
  `base_chassis_link`, which appears nowhere in the sim; the chassis link is
  called `chassis` and hangs off `base_link`. Vision pointclouds are published
  in `base_link`, so leave it alone.
- Sensors: `/scan`, `/imu`

---

## Roadmap (sim → real)

1. **UART bridge** — ROS `cmd_vel` → ESP32 (translate Twist to the firmware's commands).
2. **Wheel odometry** — parse ESP32 telemetry → publish `/odom` for EKF.
3. **Real sensors** — RPLidar `/scan` (done) + IMU, replacing Gazebo.
4. **SLAM on hardware** — slam_toolbox on the real `/scan`.
5. **Nav2 on hardware** — goals → path → drive, `use_sim_time=false`.
6. **Autonomous cone slalom** — the competition task.

See `ACHILLES_HARDWARE_PROGRESS.md` for exactly where each stands and the decisions still open.
