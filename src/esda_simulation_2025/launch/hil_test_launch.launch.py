#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'esda_simulation_2025'

    # ------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_teleop = LaunchConfiguration('launch_teleop')
    left_port = LaunchConfiguration('left_port')
    right_port = LaunchConfiguration('right_port')
    gui_host = LaunchConfiguration('gui_host')
    gui_port = LaunchConfiguration('gui_port')
    max_forward_rpm = LaunchConfiguration('max_forward_rpm')
    max_reverse_rpm = LaunchConfiguration('max_reverse_rpm')


    # ------------------------------------------------------------
    # Robot State Publisher
    #
    # Loads the URDF and publishes the robot link transforms.
    # No Gazebo, no LiDAR.
    # ------------------------------------------------------------

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': 'false',
            'use_lidar': 'false',
        }.items()
    )


    # ------------------------------------------------------------
    # Joint State Publisher
    #
    # Publishes default positions for movable joints so that
    # robot_state_publisher can publish transforms for the wheels.
    # Measured wheel positions from the ESP32 bridge are merged in
    # via source_list, so the wheels spin in RViz.
    # ------------------------------------------------------------

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'publish_default_positions': True,
                'rate': 30.0,
                'source_list': ['/esp32/joint_states'],
            }
        ]
    )


    # ------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'view_bot.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {
                'use_sim_time': use_sim_time
            }
        ],
        condition=IfCondition(launch_rviz)
    )


    # ------------------------------------------------------------
    # WASD Teleop
    #
    # This script publishes:
    #
    #   geometry_msgs/msg/Twist
    #            ↓
    #         /cmd_vel
    #
    # xterm is used because your teleop script reads keyboard
    # input directly from stdin.
    # ------------------------------------------------------------

    teleop_script = 'src/esda_simulation_2025/scripts/teleop_wasd.py'

    teleop = ExecuteProcess(
        cmd=[
            'xterm',
            '-T',
            'ESDA WASD Teleop',
            '-e',
            'python3',
            teleop_script
        ],
        output='screen',
        condition=IfCondition(launch_teleop)
    )


    # ------------------------------------------------------------
    # ESP32 Wheel Bridge
    #
    # Subscribes to:
    #
    #   /cmd_vel
    #
    # converts it to left/right wheel RPM and sends
    # "VELOCITY <rpm>" over USB serial to the two
    # wheel_controller ESP32s. Ports are auto-detected from the
    # firmware's READY / CONFIG output unless set explicitly.
    #
    # Also serves the web GUI for setting velocities:
    #
    #   http://<gui_host>:<gui_port>
    #
    # ------------------------------------------------------------

    esp32_wheel_bridge = Node(
        package=package_name,
        executable='esp32_wheel_bridge.py',
        name='esp32_wheel_bridge',
        output='screen',
        parameters=[
            {
                'left_port': left_port,
                'right_port': right_port,
                'gui_host': gui_host,
                'gui_port': ParameterValue(gui_port, value_type=int),
                'max_forward_rpm': ParameterValue(max_forward_rpm, value_type=float),
                'max_reverse_rpm': ParameterValue(max_reverse_rpm, value_type=float),
                'wheel_radius': 0.1625,
                'wheel_separation': 0.5,
                'cmd_vel_timeout': 0.5,
            }
        ]
    )

    odom_script = 'src/esda_simulation_2025/scripts/cmd_vel_odometry.py'

    cmd_vel_odometry = ExecuteProcess(
        cmd=[
            'python3',
            odom_script
        ],
        output='screen'
    )

    # ------------------------------------------------------------
    # Launch Description
    # ------------------------------------------------------------

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'launch_teleop',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'left_port',
            default_value='auto',
            description='Serial port of the left ESP32, e.g. /dev/ttyACM0 (auto = detect)'
        ),

        DeclareLaunchArgument(
            'right_port',
            default_value='auto',
            description='Serial port of the right ESP32, e.g. /dev/ttyACM1 (auto = detect)'
        ),

        DeclareLaunchArgument(
            'gui_host',
            default_value='127.0.0.1',
            description='Web GUI bind address (0.0.0.0 to reach it from another machine)'
        ),

        DeclareLaunchArgument(
            'gui_port',
            default_value='8767'
        ),

        DeclareLaunchArgument(
            'max_forward_rpm',
            default_value='35.0'
        ),

        DeclareLaunchArgument(
            'max_reverse_rpm',
            default_value='13.0'
        ),

        # Robot model
        rsp,

        # Joint states for wheel transforms
        joint_state_publisher,

        # /cmd_vel -> wheel RPM -> ESP32s, plus web GUI
        esp32_wheel_bridge,

        cmd_vel_odometry,

        # RViz
        rviz,

        # WASD keyboard control
        teleop,
    ])