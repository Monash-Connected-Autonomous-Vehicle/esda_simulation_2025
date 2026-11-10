## 2025 ESDA Vehicle Simulation
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

#### 2.5. Real Robot (Jetson)
To run the real robot with SLAM, Nav2 and lane detection, from the UI or from a terminal over SSH, see [REAL_ROBOT.md](src/esda_simulation_2025/REAL_ROBOT.md).

### 3. TO-DO
#### 3.1 Tune follow_the_gap.py
- This is a local controller. It's job is to read /scan messages, avoid obstacles, avoid lane edges should they appear as obstacles / map costs, publish /cmd_vel

#### 3.2 Write track_follower.py
- This is the brains behind the local controller. It tells the local controller which way course is going, prefer to go left / right / straight, where lane secntre is, whether an opening is actually the road or just empty space off the course

#### 3.3 Lane / Corridor perception source
- Something that gives course structure. Usually lane markers from camera, occupancy gripd / costmap, cone / lane obstacle detections, centreline estimation

#### 3.4 State machine / behaviour tree logic
- Logic above everything else that decised the mode:
    - Normal Track Following - lane bias + FTG
    - Obstacle avoidance - FTG dominates
    - Blocked / dead end / no safe gap - stop, creep, recover
    - Start-up / map not ready / TF missing - Hold still
    - Goal / waypoint mode - Maybe NAV2 takes over
    - Recovery - NAV2