## 2025 ESDA Vehicle Simulation
### 1. Set up
#### 1.1. Install Necessary Dependencies

#### 1.2. Building the Project
- Please clone the repository into the `src` folder of your work space, your folder structure should look like this:
```
|_workspace_name
    |_src
        |_2025_esda_simulation
```
- When you are trying to build the project, please do it in the `\workspace_name` level. So the `log`, `build` and `install` folders are generated next to the `src` folder. 
```
|_workspace_name
    |_src
        |_2025_esda_simulation
    |_install
    |_build
    |_log
```
- To source your local build into the current path, so it can be seen by ros2.
```
source install/setup.bash
```

### 2. Launching Project
#### 2.1. Teleop
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped
```
#### 2.2. Launch slam_toolbox (Mapping Mode)
```
ros2 launch slam_toolbox online_async_launch.py
```
#### 2.3. Launch AMCL Localization (Navigation Mode)
```
ros2 launch nav2_bringup localization_launch.py map:=./my_map_save.yaml 
```
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true 

```