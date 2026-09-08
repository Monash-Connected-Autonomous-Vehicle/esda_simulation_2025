# Dependencies & Running on Windows via Docker

## 1. Dependency list

Target platform is **Ubuntu 22.04 + ROS 2 Humble**. Nothing in this repo runs natively on Windows — the Docker image below is the supported way to use it from a Windows host.

### 1.1 Core platform

| Dependency | Why |
|---|---|
| ROS 2 Humble (`desktop-full`) | Base distro; pulls in `rclpy`, `rclcpp`, `rviz2`, common interfaces |
| **Gazebo Fortress** (`ignition-fortress`) | `worlds/*.sdf` load `libignition-gazebo-*-system.so` plugins — Fortress ABI specifically, *not* Garden or Harmonic |
| `ros-gz`, `ros-gz-sim`, `ros-gz-bridge`, `ros-gz-image` | `launch_sim.launch.py` spawns via `ros_gz_sim/gz_sim.launch.py` and bridges `/scan`, `/odom`, `/cmd_vel`, `/clock`, `/camera/*` |
| `python3-colcon-common-extensions` | Workspace build |

### 1.2 Control

| Dependency | Why |
|---|---|
| `ros2-control`, `ros2-controllers` | `diff_drive_base_controller`, `joint_state_broadcaster` |
| `controller-manager` | `ros2_control_node` on the real robot; spawners in sim |
| `gz-ros2-control` | `gz_ros2_control-system` plugin in `description/ros2_control_2wd.xacro` |
| `libserial-dev` | **`esda_hardware_2025` will not build without it** — `stm_comms.hpp` includes `<libserial/SerialPort.h>` and CMake does `target_link_libraries(... serial)` |

### 1.3 Navigation & mapping

| Dependency | Why |
|---|---|
| `navigation2`, `nav2-bringup`, `nav2-msgs` | `navigation_launch.py`, `localization_launch.py`, `nav2_params.yaml` |
| `nav2-simple-commander` | `waypoint_navigator*.py` import `BasicNavigator`, `TaskResult` |
| `slam-toolbox` | `online_async_launch.py` mapping |
| `robot-localization` | `robot_localization_ekf.launch.py`, `config/ekf.yaml` |
| `xacro`, `robot-state-publisher`, `joint-state-publisher[-gui]` | URDF pipeline |

### 1.4 Perception & TF

| Dependency | Why |
|---|---|
| `cv-bridge`, `python3-opencv` | All three lane detectors |
| `sensor-msgs-py` | `sensor_msgs_py.point_cloud2` in the waypoint navigator |
| `tf2-ros`, `tf2-geometry-msgs` | `do_transform_point`, TF buffers throughout |
| `tf-transformations` + `python3-transforms3d` | `import tf_transformations` in `behaviour_tree.py` — a separate package from `tf2_ros`, easy to miss |
| `pcl-conversions` | Declared in `package.xml` |
| `tf2-tools` | `ros2 run tf2_tools view_frames` (the committed `frames_*.pdf` came from this) |

### 1.5 Real robot only

`velodyne`, `velodyne-driver`, `velodyne-pointcloud`, `velodyne-laserscan` (VLP-16 via `config/vlp16.yaml`), plus `joy` and `teleop-twist-joy` for the gamepad in `launch_robot.launch.py`.

### 1.6 Teleop & UI

| Dependency | Why |
|---|---|
| `python3-tk` + `customtkinter` (pip) | `ui_launch.py` / `ui_launch_real_robot.py` |
| `xterm` | The UI runs every module in its own xterm — it warns and degrades without it |
| `psmisc`, `procps` | `killall` / `pkill` in "Kill All Processes" |
| `teleop-twist-keyboard` | Documented teleop path |

### 1.7 Optional — alternative lane detectors

Both are **off by default** in the image; the classic CV detector (`lane_detection.py`) needs neither.

| Detector | Needs | Extra manual step |
|---|---|---|
| `lane_detection_FCN.py` | TensorFlow/Keras | `lane-detection-on-rural-roads-master/` checkout with `FCN_model.h5` |
| `lane_detection_twinlite.py` | PyTorch (CPU ok) | `git clone https://github.com/chequanghuy/TwinLiteNetPlus.git` **plus** a `.pth` from its Google Drive link into `TwinLiteNetPlus/pretrained/` |

Both directories are gitignored and live *outside* the ROS package. See `src/esda_simulation_2025/LANE_DETECTION.md`.

---

## 2. Prerequisites on Windows

- **Docker Desktop** with the **WSL2 backend** enabled.
- **Windows 11** gives you WSLg, so GUI apps (Gazebo, RViz, the Tkinter launcher, OpenCV windows) work with no extra X server.
- **Windows 10**: install an X server ([VcXsrv](https://sourceforge.net/projects/vcxsrv/), "Disable access control" checked) and set `DISPLAY=host.docker.internal:0.0` in `docker-compose.yml`.
- Clone the repo **inside the WSL2 filesystem** (e.g. `\\wsl$\Ubuntu\home\<you>\esda_simulation_2025`), not under `C:\Users\...`. Bind-mounting across the Windows/WSL boundary makes `colcon build` several times slower.

## 3. Build and run

```bash
docker compose build
```

```bash
docker compose run --rm esda
```

Inside the container:

```bash
colcon build --symlink-install && source install/setup.bash
```

```bash
python3 src/esda_simulation_2025/scripts/ui_launch.py
```

To include the optional detectors, build with:

```bash
docker compose build --build-arg INSTALL_TORCH=true --build-arg INSTALL_TENSORFLOW=true
```

## 4. What the image already handles

- **Fast DDS shared memory** — unreliable across the Docker/WSL boundary. `FASTRTPS_DEFAULT_PROFILES_FILE` is preset to the repo's own `config/fastdds_noshm.xml` (UDP-only), which is the same workaround `ui_launch.py` applies per-terminal.
- **`shm_size: 2gb`** — Gazebo plus Nav2 plus RViz will exhaust Docker's 64 MB default.
- **`LIBGL_ALWAYS_SOFTWARE=1`** — set in compose because Docker Desktop on Windows gives no GPU for OpenGL. Gazebo rendering will be slow; drop the sim's real-time factor expectations accordingly. Comment it out on native Linux with working GPU passthrough.
- **`GZ_SIM_RESOURCE_PATH`** — preset so the `worlds/models/orange_igvc` meshes resolve.

## 5. Known caveats

- The image builds `src/` at image-build time, but `docker-compose.yml` bind-mounts the repo over `/ros2_ws`, which shadows that prebuilt `install/`. Run `colcon build` once inside the container after first start. (Drop the `.:/ros2_ws` volume if you want the baked-in build instead.)
- `build/`, `install/`, `log/` produced inside the container land in your working tree. They are gitignored, but they are Linux artefacts — delete them before building anywhere else.
- Serial hardware (`esda_hardware_2025` talking to the STM) needs a USB device passed through. Docker Desktop on Windows cannot do this directly; use `usbipd-win` to attach the device to WSL2 first, then add a `devices:` entry to the compose service.
- No GPU means the TwinLiteNet+ detector runs CPU-only, roughly 8–9 Hz on the `nano` variant.
