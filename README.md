## 2025 ESDA Vehicle Simulation

### Branches
| Branch | Purpose | Owner | Status |
|---|---|---|---|
| `main` | Stable, default branch | @sam | ✅ Stable |
| `yolo-lane-cone` | YOLO detection for lanes and cones | @felix | 🚧 Active |
| `feature/nav2-recovery` | Nav2 recovery behaviours - waypoint navigator recommendation | @sam | 🚧 Active |
| `yolo-adaptive` | Adaptive YOLO perception | @felix | 🚧 Active |
| `feature/hardware-interface` | ros2_control hardware interface for the real vehicle | @xavier-marie | 🚧 Active |
| `feature/new-maps` | New simulation maps/worlds | @ishaan | 🚧 Active |
| `esda-jetson-testing` | Testing on the Jetson Orin Nano | @jun-hau | 🚧 Active |
| `feature/reactive-local-navigation` | Reactive local planner / obstacle avoidance | @ishaan | 💤 Paused |
| `feature/waypoint-recommender-revamp` | Waypoint recommender rework (PR #1) | @ishaan | 🔍 In review |

Legend: ✅ Stable · 🚧 Active · 🔍 In review · 💤 Paused

#### Archived branches
`sam`, `victor`, `mattlab_pc`, `xavier`, `xavier_linux_asus_laptop` are archived as tags (`archive/<name>`).
To restore one: `git fetch --tags && git checkout -b <name> archive/<name>`

#### Archived branches
`sam`, `victor`, `mattlab_pc`, `xavier`, `xavier_linux_asus_laptop` are archived as tags (`archive/<name>`).
To restore one: `git fetch --tags && git checkout -b <name> archive/<name>`

> To use a branch: `git checkout <branch>`, then rebuild from `workspace_name` and run `source install/setup.bash`.
### 1. Set up
#### 1.1. Install Necessary Dependencies

#### 1.2. Building the Project
- Please clone the repository into the `src` folder of your work space, your folder structure should look like this:
```
|_workspace_name
    |_src
        |_esda_simulation_2025
```
- When you are trying to build the project, please do it in the `\workspace_name` level. So the `log`, `build` and `install` folders are generated next to the `src` folder. 
```
|_workspace_name
    |_src
        |_esda_simulation_2025
    |_install
    |_build
    |_log
```
- To source your local build into the current path, so it can be seen by ros2.
```
source install/setup.bash
```

### 2. Launching Project
#### 2.1. Launch Simulator
```
ros2 launch esda_simulation_2025 launch_sim.launch.py
```
#### 2.2. Teleop
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped
```
#### 2.3. Launch slam_toolbox (Mapping Mode)
```
ros2 launch slam_toolbox online_async_launch.py
```
#### 2.4. Launch AMCL Localization + NAV2 (Navigation Mode)
```
ros2 launch esda_simulation_2025 localization_launch.py
```
```
ros2 launch esda_simulation_2025 navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true 

```
