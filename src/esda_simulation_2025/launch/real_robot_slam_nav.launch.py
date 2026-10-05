#!/usr/bin/env python3

# One-shot real-robot bring-up: everything the GUI's
# "Launch Real Robot (ODrive + LiDAR)" -> "Launch SLAM" -> "Launch Nav2" ->
# "WASD Teleop" sequence starts, with the usual mounts:
#
#   ZED high on the pole (camera_mount:=pole_top), LiDAR low on the
#   chassis (lidar_mount:=low)
#   VLP-16 + ODrive bridge + throttled /velodyne_points_viz for Foxglove
#   slam_toolbox on /scan (map -> odom)
#   Nav2 on /scan + /odom (starts 8 s after the rest)
#   WASD teleop in its own xterm
#   no RViz (use Foxglove, or pass launch_rviz:=true)
#
#     ros2 launch esda_simulation_2025 real_robot_slam_nav.launch.py
#
# Any launch_odrive_robot.launch.py argument can still be overridden, e.g.
# launch_teleop:=false launch_joy:=true for the gamepad instead of WASD.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_share = get_package_share_directory('esda_simulation_2025')

    defaults = {
        'camera_mount': 'pole_top',   # ZED high, on the pole
        'lidar_mount': 'low',         # LiDAR low, directly on the chassis
        'launch_lidar': 'true',
        'launch_odrive': 'true',
        'launch_slam': 'true',
        'launch_nav2': 'true',
        'launch_teleop': 'true',
        'launch_joy': 'false',
        'launch_rviz': 'false',
    }

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'launch_odrive_robot.launch.py')
        ),
        launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items(),
    )

    # Same UDP-only DDS profile as the GUI and the Foxglove bridge service, so
    # every node is visible to Foxglove and to ros2 CLI tools no matter which
    # shell this is launched from.
    dds_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(pkg_share, 'config', 'fastdds_noshm.xml'))

    return LaunchDescription(
        [dds_profile]
        + [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [robot]
    )
