#!/usr/bin/env python3

# Real-robot bring-up for the Jetson: Velodyne VLP-16 over ethernet + ODrive
# v3.6 over USB + RViz, optionally SLAM Toolbox and Nav2. No ros2_control,
# no Gazebo.
#
#   /cmd_vel -> odrive_bridge.py -> ODrive axis0/axis1
#   ODrive encoders -> /odom + odom->base_link TF + /joint_states
#   VLP-16 -> /velodyne_packets -> /velodyne_points -> /scan
#   /scan -> slam_toolbox -> map -> odom;  /scan + /odom -> Nav2 -> /cmd_vel
#
# The Jetson must have an IPv4 address on the LiDAR's subnet or the kernel
# drops the VLP-16's UDP packets, e.g. for a factory-default sensor:
#     sudo ip addr add 192.168.1.100/24 dev enP8p1s0
#
# Example:
#     ros2 launch esda_simulation_2025 launch_odrive_robot.launch.py
#     ros2 launch esda_simulation_2025 launch_odrive_robot.launch.py launch_slam:=true launch_nav2:=true

import math
import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'esda_simulation_2025'
    pkg_share = get_package_share_directory(package_name)

    lidar_mount = LaunchConfiguration('lidar_mount')
    camera_mount = LaunchConfiguration('camera_mount')
    launch_lidar = LaunchConfiguration('launch_lidar')
    lidar_ip = LaunchConfiguration('lidar_ip')
    launch_odrive = LaunchConfiguration('launch_odrive')
    odrive_serial = LaunchConfiguration('odrive_serial')
    gear_ratio = LaunchConfiguration('gear_ratio')
    max_motor_turns_per_s = LaunchConfiguration('max_motor_turns_per_s')
    left_direction = LaunchConfiguration('left_direction')
    right_direction = LaunchConfiguration('right_direction')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_teleop = LaunchConfiguration('launch_teleop')
    launch_slam = LaunchConfiguration('launch_slam')
    launch_nav2 = LaunchConfiguration('launch_nav2')

    # ------------------------------------------------------------
    # Robot State Publisher
    #
    # URDF with the LiDAR included, so laser_frame sits on the pole and the
    # point cloud lines up with the robot model. Wheel joint angles come
    # from odrive_bridge.py on /joint_states.
    # ------------------------------------------------------------

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'false',
            'use_lidar': 'true',
            'lidar_mount': lidar_mount,
            'camera_mount': camera_mount,
        }.items()
    )

    # ------------------------------------------------------------
    # ODrive bridge
    # ------------------------------------------------------------

    odrive_bridge = Node(
        package=package_name,
        executable='odrive_bridge.py',
        name='odrive_bridge',
        output='screen',
        parameters=[
            {
                'serial_number': ParameterValue(odrive_serial, value_type=str),
                'gear_ratio': ParameterValue(gear_ratio, value_type=float),
                'max_motor_turns_per_s': ParameterValue(max_motor_turns_per_s, value_type=float),
                'left_direction': ParameterValue(left_direction, value_type=float),
                'right_direction': ParameterValue(right_direction, value_type=float),
                'wheel_radius': 0.1625,
                'wheel_separation': 0.5,
                'cmd_vel_timeout': 0.5,
                'publish_odom_tf': True,
            }
        ],
        condition=IfCondition(launch_odrive),
    )

    # Without the ODrive nothing publishes odom -> base_link, so pin the
    # robot at the origin to still see the LiDAR in RViz.
    static_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
        output='screen',
        condition=UnlessCondition(launch_odrive),
    )

    # ------------------------------------------------------------
    # Velodyne VLP-16
    #
    # frame_id is the URDF's laser_frame (not the driver's default
    # "velodyne"), so robot_state_publisher already places the sensor and
    # no extra static transform is needed.
    # ------------------------------------------------------------

    velodyne_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='screen',
        parameters=[{
            'device_ip': ParameterValue(lidar_ip, value_type=str),
            'port': 2368,
            'model': 'VLP16',
            'rpm': 600.0,
            'frame_id': 'laser_frame',
            'gps_time': False,
        }],
        condition=IfCondition(launch_lidar),
    )

    velodyne_pointcloud_share = get_package_share_directory('velodyne_pointcloud')

    velodyne_transform = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        output='screen',
        parameters=[{
            'calibration': os.path.join(velodyne_pointcloud_share, 'params', 'VLP16db.yaml'),
            'model': 'VLP16',
            # The sensor sits on a 1 m pole above the chassis, so returns
            # off the robot itself are already mostly out of view.
            'min_range': 0.4,
            'max_range': 100.0,
            'fixed_frame': '',
            'target_frame': '',
            'organize_cloud': True,
        }],
        condition=IfCondition(launch_lidar),
    )

    # 3D cloud -> 2D /scan for SLAM/Nav2/FTG.
    #
    # pointcloud_to_laserscan flattens every return within a height band
    # (in base_link, which sits at axle height - the ground is z = -0.1625)
    # into one scan. That's preferred over velodyne_laserscan's single ring:
    # from the 1.4 m pole a near-horizontal ring passes over cones and
    # anything else shorter than the pole. Both are optional apt packages,
    # so fall back gracefully rather than fail the launch.
    def package_exists(name):
        try:
            get_package_share_directory(name)
            return True
        except PackageNotFoundError:
            return False

    if package_exists('pointcloud_to_laserscan'):
        cloud_to_scan = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[('cloud_in', '/velodyne_points'), ('scan', '/scan')],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.05,
                'min_height': -0.05,  # ~0.11 m above the ground
                'max_height': 1.3,
                'angle_min': -math.pi,
                'angle_max': math.pi,
                'angle_increment': math.radians(0.4),
                'scan_time': 0.1,
                'range_min': 0.45,  # inside robot_radius - drop hits on the robot itself
                'range_max': 30.0,
                'use_inf': True,
            }],
            condition=IfCondition(launch_lidar),
        )
    elif package_exists('velodyne_laserscan'):
        cloud_to_scan = Node(
            package='velodyne_laserscan',
            executable='velodyne_laserscan_node',
            name='velodyne_laserscan',
            output='screen',
            remappings=[('velodyne_points', '/velodyne_points'), ('scan', '/scan')],
            parameters=[{'ring': 8, 'resolution': 0.007}],
            condition=IfCondition(launch_lidar),
        )
    else:
        cloud_to_scan = LogInfo(
            msg='No cloud->scan converter installed - no /scan, so SLAM/Nav2 have no '
                'LiDAR. Install with: sudo apt install ros-humble-pointcloud-to-laserscan'
        )

    # ------------------------------------------------------------
    # SLAM Toolbox / Nav2 (optional - the GUI's SLAM and Nav2 buttons do
    # the same thing). Only resolved when enabled, so they don't need to be
    # installed to bring up the robot.
    # ------------------------------------------------------------

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'false', 'scan_topic': '/scan'}.items(),
        condition=IfCondition(launch_slam),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map_subscribe_transient_local': 'true',
            'scan_topic': '/scan',
            'odom_topic': '/odom',
        }.items(),
        condition=IfCondition(launch_nav2),
    )
    # Let the LiDAR, TF and SLAM's map come up before Nav2 starts.
    delayed_nav2 = TimerAction(period=8.0, actions=[nav2])

    # ------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'view_lidar.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(launch_rviz),
    )
    delayed_rviz = TimerAction(period=3.0, actions=[rviz])

    # ------------------------------------------------------------
    # WASD teleop (reads stdin, so it needs its own terminal)
    # ------------------------------------------------------------

    teleop = ExecuteProcess(
        cmd=[
            'xterm', '-T', 'ESDA WASD Teleop', '-e',
            'ros2', 'run', package_name, 'teleop_wasd.py',
        ],
        output='screen',
        condition=IfCondition(launch_teleop),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_mount', default_value='pole_top',
            description='LiDAR mount point: "pole_top" or "low"'),
        DeclareLaunchArgument(
            'camera_mount', default_value='front',
            description='ZED mount point: "front" or "pole_top"'),
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument(
            'lidar_ip', default_value='',
            description='VLP-16 IP to accept packets from ("" = any sender)'),
        DeclareLaunchArgument('launch_odrive', default_value='true'),
        DeclareLaunchArgument(
            'odrive_serial', default_value='',
            description='ODrive serial number in hex ("" = first one found)'),
        DeclareLaunchArgument(
            'gear_ratio', default_value='64.0',
            description='Motor turns per wheel turn'),
        DeclareLaunchArgument(
            'max_motor_turns_per_s', default_value='40.0',
            description='Motor-side speed clamp; 40 turns/s = ~0.64 m/s at 64:1'),
        DeclareLaunchArgument(
            'left_direction', default_value='1.0',
            description='1.0 or -1.0; flip if the left wheel spins backwards'),
        DeclareLaunchArgument(
            'right_direction', default_value='-1.0',
            description='1.0 or -1.0; flip if the right wheel spins backwards'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('launch_teleop', default_value='true'),
        DeclareLaunchArgument(
            'launch_slam', default_value='false',
            description='Run slam_toolbox (map -> odom) on /scan'),
        DeclareLaunchArgument(
            'launch_nav2', default_value='false',
            description='Run Nav2 on /scan + /odom (needs SLAM or AMCL for map -> odom)'),

        rsp,
        odrive_bridge,
        static_odom,

        velodyne_driver,
        velodyne_transform,
        cloud_to_scan,

        slam,
        delayed_nav2,

        delayed_rviz,
        teleop,
    ])
