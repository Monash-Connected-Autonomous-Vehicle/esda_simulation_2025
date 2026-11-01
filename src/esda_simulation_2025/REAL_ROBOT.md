# Running the Real Robot

How to run Asterius on the Jetson with SLAM and Nav2 and drive it to a waypoint.

## Overview
The robot uses a Velodyne VLP-16 over ethernet and an ODrive v3.6 over USB. There is no Gazebo or `ros2_control`.

```text
Wheels:   /cmd_vel -> odrive_bridge.py -> ODrive axis0/axis1
Odometry: ODrive encoders -> /odom + odom->base_link TF + /joint_states
LiDAR:    VLP-16 -> /velodyne_packets -> /velodyne_points -> /scan
Mapping:  /scan -> slam_toolbox -> map->odom
Driving:  /scan + /odom -> Nav2 -> /cmd_vel
```

Lane detection and the ZED are not started, so the robot only avoids what the LiDAR sees. It will not follow painted lanes.

## Before You Start
- The LiDAR is powered and its ethernet cable is plugged in.
- The ODrive is connected over USB, with its config already saved on the board. Nothing applies `odrive_configs/odrive_config.json` at launch.
- The wheels are off the ground for a first test, or someone is ready to cut power. The wheels follow any `/cmd_vel` as soon as the launch is up.
- The robot is pointing where you want it. Its starting pose becomes the map origin.

## Launch via UI (Recommended)
1. Run the UI launcher:
   ```bash
   python3 ~/Mk3/esda_simulation_2025/src/esda_simulation_2025/scripts/ui_launch.py
   ```
2. If the code has changed, click **1. Colcon Build**.
3. Click **Launch Autonomous Real Robot (SLAM + Nav2)**. Enter the `sudo` password in the xterm if asked.
4. Wait until the xterm shows Nav2's nodes are active. Nav2 starts about 8 seconds after everything else.
5. Click **Launch RViz2**, then change **Global Options > Fixed Frame** to `map`.
6. Send a waypoint (see below).

The button also brings up `enP8p1s0` and gives it `192.168.1.100/24`, which the Jetson needs to receive LiDAR packets.

Do not click these while the autonomous launch is running:
- **Launch SLAM** and **Launch Nav2**: the launch already runs both, so they just report an error.
- **Launch Real Robot (ODrive + LiDAR)**: it is blocked because both launches use the ODrive.
- **Waypoint Nav**: this is the older `waypoint_navigator.py`, which follows fixed waypoints from `igvc_waypoints.yaml`.

The button ignores the **Enable LIDAR** and joystick checkboxes, the mount dropdowns and **RViz (Real Robot)**. It always uses the LiDAR low on the chassis and the ZED on the pole, and it does not open WASD teleop.

## Launch from a Terminal
```bash
cd ~/Mk3/esda_simulation_2025
source install/setup.bash
sudo ip link set enP8p1s0 up
sudo ip addr add 192.168.1.100/24 dev enP8p1s0   # once per boot
ros2 launch esda_simulation_2025 real_robot_slam_nav.launch.py launch_teleop:=false
```

Launch from the workspace root, because `nav2_params_real_robot.yaml` finds its behaviour tree through a relative `install/...` path.

Useful arguments:

| Argument | Default | Purpose |
|---|---|---|
| `launch_teleop` | `true` | WASD teleop in its own xterm (the UI button sets this to `false`) |
| `launch_joy` | `false` | Gamepad teleop |
| `launch_rviz` | `false` | RViz on the Jetson |
| `launch_virtual_lanes` | `false` | Virtual walls either side of the start pose |
| `lane_half_width` | `2.0` | Distance in metres from the start pose to each wall |
| `camera_mount` | `pole_top` | `front` or `pole_top` |
| `lidar_mount` | `low` | `low` or `pole_top` |

## Sending a Waypoint
The robot does not move until it gets a goal. A new goal replaces the current one.

### RViz
Click **2D Goal Pose**, click the target on the map, then drag to set the final heading. The robot starts driving straight away.

### Terminal on the Jetson
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/Mk3/esda_simulation_2025/src/esda_simulation_2025/config/fastdds_jetson.xml
source ~/Mk3/esda_simulation_2025/install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}"
```

In the `map` frame, `x` is forward from the start pose and `y` is to the left.

Nav2 is capped at 0.45 m/s and treats the goal as reached within 0.25 m.

## RViz on a Laptop
RViz on the Jetson can take CPU that SLAM and Nav2 need. To run it on a laptop over Tailscale instead:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=<repo>/src/esda_simulation_2025/config/fastdds_remote_jetson.xml
source <repo>/install/setup.bash
rviz2 -d <repo>/src/esda_simulation_2025/config/view_remote.rviz
```

The addresses are hardcoded, so update them if they change:
- `fastdds_jetson.xml`: the laptop, `100.72.211.34`
- `fastdds_remote_jetson.xml`: the Jetson, `100.67.86.95`

The UI button also works with a laptop, because `real_robot_slam_nav.launch.py` sets `fastdds_jetson.xml` for its own nodes.

## Stopping
- Click the green button again, or **KILL ALL PROCESSES**.
- From a terminal, press Ctrl+C on the launch. `odrive_bridge.py` sets the axes to idle on shutdown.
- Keep a physical way to cut power, because these all stop the robot through software.

The wheels stop if `/cmd_vel` is missing for 0.5 s. Do not use WASD to stop the robot during autonomous driving: it publishes to `/cmd_vel` alongside Nav2, so the two fight. Close WASD before sending waypoints.

## Virtual Lane Walls
`virtual_lane_walls.py` adds two straight walls parallel to the robot's starting heading, enforced by Nav2's keepout filter. They are off by default. Turn them on with `launch_virtual_lanes:=true`.

- They need SLAM's `map` frame.
- They do not follow curves, so point the robot down the lane before launching.
- Do not run them with `waypoint_navigator_recommendation.py`, which uses the same `/keepout_costmap_filter_info` topic.

## Troubleshooting

| Symptom | Check |
|---|---|
| No `/scan`, or the map stays empty | Is the LiDAR powered and plugged in? `cat /sys/class/net/enP8p1s0/carrier` should print `1`. The address assumes the LiDAR's factory subnet, `192.168.1.x`. |
| Nav2 cannot load its behaviour tree | The launch was started outside the workspace root. |
| Goal accepted but the wheels do not move | Run `ros2 topic echo /cmd_vel`. If Nav2 is sending commands, look for errors from `odrive_bridge` in the launch xterm. |
| Laptop RViz shows nothing | Check the hardcoded Tailscale addresses above. |
| RViz shows the robot but no map | Set the Fixed Frame to `map`. |

## Files

| File | Purpose |
|---|---|
| `launch/real_robot_slam_nav.launch.py` | One-command launch with the usual mounts |
| `launch/launch_odrive_robot.launch.py` | Robot bring-up, with SLAM and Nav2 optional |
| `scripts/odrive_bridge.py` | `/cmd_vel` to ODrive, encoders to `/odom` |
| `scripts/ui_launch.py` | UI launcher, including the autonomous button |
| `scripts/virtual_lane_walls.py` | Virtual lane walls |
| `config/nav2_params_real_robot.yaml` | Nav2 settings for the real robot |
| `config/fastdds_jetson.xml` | DDS profile for the Jetson |
| `config/fastdds_remote_jetson.xml` | DDS profile for a laptop |
