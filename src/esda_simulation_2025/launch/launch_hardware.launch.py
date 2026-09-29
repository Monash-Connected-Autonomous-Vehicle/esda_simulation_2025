#!/usr/bin/env python3

# Real-hardware bring-up: Velodyne LiDAR + ZED stereo camera + ESP32 (over
# serial, via the esda_hardware_2025 ros2_control plugin) + Nav2 +
# waypoint_navigator_recommendation.py. No Gazebo - this is the hardware
# counterpart of launch_sim.launch.py.
#
# Requires overlaying two extra workspaces that this repo does not build
# itself:
#     source install/setup.bash                       # this workspace
#     source /home/esda/Workspaces/install/setup.bash  # velodyne + zed_wrapper
# (/home/esda/esda_ws has dangling symlinks for zed_wrapper as of this
# writing - use /home/esda/Workspaces instead.)
#
# Example:
#     ros2 launch esda_simulation_2025 launch_hardware.launch.py \
#         serial_port:=/dev/ttyUSB0 slam_mode:=true

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'esda_simulation_2025'
    pkg_share = get_package_share_directory(package_name)

    # ---- launch arguments --------------------------------------------

    serial_port = LaunchConfiguration('serial_port')
    serial_baud_rate = LaunchConfiguration('serial_baud_rate')
    camera_mount = LaunchConfiguration('camera_mount')
    lidar_mount = LaunchConfiguration('lidar_mount')

    launch_lidar = LaunchConfiguration('launch_lidar')

    launch_zed = LaunchConfiguration('launch_zed')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    zed_left_topic = LaunchConfiguration('zed_left_topic')
    zed_right_topic = LaunchConfiguration('zed_right_topic')
    zed_depth_topic = LaunchConfiguration('zed_depth_topic')
    zed_imu_topic = LaunchConfiguration('zed_imu_topic')

    launch_lane_detection = LaunchConfiguration('launch_lane_detection')
    lane_detector_mode = LaunchConfiguration('lane_detector_mode')
    scan_topic = LaunchConfiguration('scan_topic')

    slam_mode = LaunchConfiguration('slam_mode')
    map_yaml = LaunchConfiguration('map')

    launch_nav2 = LaunchConfiguration('launch_nav2')
    launch_waypoint_navigator = LaunchConfiguration('launch_waypoint_navigator')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # ---- robot_description + ros2_control (ESP32 over serial) --------

    xacro_file = PathJoinSubstitution(
        [FindPackageShare(package_name), 'description', 'robot.urdf.xacro']
    )

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' sim_mode:=false',
            ' use_ros2_control:=true',
            ' use_lidar:=true',
            ' serial_device:=', serial_port,
            ' serial_baud_rate:=', serial_baud_rate,
            ' camera_mount:=', camera_mount,
            ' lidar_mount:=', lidar_mount,
        ]),
        value_type=str,
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_description}],
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'my_controllers_2wd.yaml']
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': robot_description},
            robot_controllers,
        ],
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file', robot_controllers,
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager, on_start=[joint_broad_spawner])
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager, on_start=[diff_drive_spawner])
    )

    # ---- Velodyne LiDAR -> /scan --------------------------------------

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne'),
                'launch', 'velodyne-all-nodes-VLP16-launch.py',
            )
        ),
        condition=IfCondition(launch_lidar),
    )

    # The velodyne driver publishes in a "velodyne" frame that isn't in the
    # URDF (lidar.xacro only defines "laser_frame"). Mirrors launch_robot.launch.py.
    velodyne_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'velodyne'],
        output='screen',
        condition=IfCondition(launch_lidar),
    )

    velodyne_to_scan = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        name='velodyne_laserscan',
        output='screen',
        remappings=[('velodyne_points', '/velodyne_points'), ('scan', '/scan')],
        parameters=[{'ring': 8, 'resolution': 0.007}],
        condition=IfCondition(launch_lidar),
    )

    # ---- ZED camera -----------------------------------------------------

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch', 'zed_camera.launch.py',
            )
        ),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': camera_model,
            # robot_state_publisher (from the URDF) + the EKF below own the
            # TF tree; don't let the ZED node publish its own odom/map TF.
            'publish_tf': 'false',
            'publish_map_tf': 'false',
        }.items(),
        condition=IfCondition(launch_zed),
    )

    # lane_detection.py hardcodes /camera/left,right,depth/image_raw; the ZED
    # wrapper publishes under /zed/zed_node/... - relay rather than edit the
    # detector. Verify these topic names with `ros2 topic list` once the ZED
    # node is up and override via the zed_*_topic args if your wrapper
    # version differs.
    zed_and_lane = PythonExpression(
        ["'", launch_zed, "' == 'true' and '", launch_lane_detection, "' == 'true'"]
    )

    imu_relay = Node(
        package='topic_tools', executable='relay', name='zed_imu_relay',
        arguments=[zed_imu_topic, '/imu/data'],
        output='screen',
        condition=IfCondition(launch_zed),
    )
    left_relay = Node(
        package='topic_tools', executable='relay', name='zed_left_relay',
        arguments=[zed_left_topic, '/camera/left/image_raw'],
        output='screen',
        condition=IfCondition(zed_and_lane),
    )
    right_relay = Node(
        package='topic_tools', executable='relay', name='zed_right_relay',
        arguments=[zed_right_topic, '/camera/right/image_raw'],
        output='screen',
        condition=IfCondition(zed_and_lane),
    )
    depth_relay = Node(
        package='topic_tools', executable='relay', name='zed_depth_relay',
        arguments=[zed_depth_topic, '/camera/depth/image_raw'],
        output='screen',
        condition=IfCondition(zed_and_lane),
    )

    # ---- EKF: fuses wheel /odom (diff_drive_base_controller, open-loop) --
    # ---- with the ZED IMU into odom->base_link TF, same role it plays in --
    # ---- launch_sim.launch.py. -------------------------------------------

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'),
            {'use_sim_time': False},  # ekf.yaml itself hardcodes true for the sim path
        ],
    )
    delayed_ekf = TimerAction(period=5.0, actions=[ekf_node])

    # ---- lane detection: publishes /lane_markers and /scan_fused --------

    lane_detection_regular = Node(
        package=package_name, executable='lane_detection.py', name='lane_detection',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", lane_detector_mode, "' == 'Regular'"])
        ),
    )
    lane_detection_fcn = Node(
        package=package_name, executable='lane_detection_FCN.py', name='lane_detection',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", lane_detector_mode, "' == 'FCN'"])
        ),
    )
    delayed_lane_detection = TimerAction(
        period=6.0,
        actions=[lane_detection_regular, lane_detection_fcn],
        condition=IfCondition(launch_lane_detection),
    )

    # ---- SLAM (build a map) or AMCL (localize against a saved map) ------
    # Both configs already default to plain base_link/odom frames (the
    # my_robot_1/ prefix only exists in Gazebo), so nothing to override here.

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'false', 'scan_topic': scan_topic}.items(),
        condition=IfCondition(slam_mode),
    )
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_yaml,
            'scan_topic': scan_topic,
        }.items(),
        condition=UnlessCondition(slam_mode),
    )
    delayed_slam = TimerAction(period=8.0, actions=[slam_launch])
    delayed_amcl = TimerAction(period=8.0, actions=[amcl_launch])

    # ---- Nav2 -------------------------------------------------------------

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map_subscribe_transient_local': 'true',
            'scan_topic': scan_topic,
        }.items(),
        condition=IfCondition(launch_nav2),
    )
    delayed_nav2 = TimerAction(period=11.0, actions=[nav2_launch])

    # Nav2's controller_server ultimately publishes plain "cmd_vel"; the real
    # diff-drive controller listens on
    # /diff_drive_base_controller/cmd_vel_unstamped (see CLAUDE.md - in sim
    # the gz bridge consumes /cmd_vel directly, so this remap only exists on
    # hardware).
    cmd_vel_relay = Node(
        package='topic_tools', executable='relay', name='nav2_cmd_vel_relay',
        arguments=['/cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped'],
        output='screen',
        condition=IfCondition(launch_nav2),
    )
    delayed_cmd_vel_relay = TimerAction(period=11.0, actions=[cmd_vel_relay])

    # ---- waypoint navigator -----------------------------------------------
    # Only one navigation stack may own /cmd_vel at a time - do not also run
    # follow_the_gap.py / track_follower.py / behaviour_tree.py alongside this.

    waypoint_navigator = Node(
        package=package_name,
        executable='waypoint_navigator_recommendation.py',
        name='waypoint_navigator',
        output='screen',
        condition=IfCondition(launch_waypoint_navigator),
    )
    delayed_waypoint_navigator = TimerAction(period=15.0, actions=[waypoint_navigator])

    # ---- RViz ---------------------------------------------------------

    rviz_config = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'view_bot.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(launch_rviz),
    )
    delayed_rviz = TimerAction(period=4.0, actions=[rviz_node])

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description="ESP32 serial device (was /dev/ttyACM0 for the STM32; "
                        "ESP32 dev boards usually enumerate as ttyUSB*)"),
        DeclareLaunchArgument(
            'serial_baud_rate', default_value='57600',
            description='Must match the ESP32 firmware Serial.begin() rate'),
        DeclareLaunchArgument(
            'camera_mount', default_value='front',
            description='ZED mount point: "front" (low, forward) or "pole_top" (on the LiDAR pole)'),
        DeclareLaunchArgument(
            'lidar_mount', default_value='pole_top',
            description='LiDAR mount point: "pole_top" (on the mounting pole) or "low" (original, directly on chassis)'),

        DeclareLaunchArgument('launch_lidar', default_value='true'),

        DeclareLaunchArgument('launch_zed', default_value='true'),
        DeclareLaunchArgument('camera_name', default_value='zed'),
        DeclareLaunchArgument(
            'camera_model', default_value='zed2i',
            description='zed2i or zed2, depending on which ZED this robot has'),
        DeclareLaunchArgument('zed_left_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('zed_right_topic', default_value='/zed/zed_node/right/image_rect_color'),
        DeclareLaunchArgument('zed_depth_topic', default_value='/zed/zed_node/depth/depth_registered'),
        DeclareLaunchArgument('zed_imu_topic', default_value='/zed/zed_node/imu/data'),

        DeclareLaunchArgument('launch_lane_detection', default_value='true'),
        DeclareLaunchArgument(
            'lane_detector_mode', default_value='FCN', description='Regular or FCN'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan_fused',
            description='Switch to /scan if launch_lane_detection:=false'),

        DeclareLaunchArgument(
            'slam_mode', default_value='true',
            description='true = build a map with SLAM, false = localize with AMCL against `map`'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_share, 'maps', 'my_map_save.yaml'),
            description='Map yaml used when slam_mode:=false'),

        DeclareLaunchArgument('launch_nav2', default_value='true'),
        DeclareLaunchArgument('launch_waypoint_navigator', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),

        rsp_node,
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,

        velodyne_launch,
        velodyne_tf,
        velodyne_to_scan,

        zed_launch,
        imu_relay,
        left_relay,
        right_relay,
        depth_relay,

        delayed_ekf,
        delayed_lane_detection,

        delayed_slam,
        delayed_amcl,
        delayed_nav2,
        delayed_cmd_vel_relay,
        delayed_waypoint_navigator,

        delayed_rviz,
    ])
