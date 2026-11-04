#!/usr/bin/env python3

# Real-robot bring-up for the Jetson: Velodyne VLP-16 over ethernet + ODrive
# v3.6 over USB, optionally RViz (launch_rviz:=true), SLAM Toolbox and Nav2. No ros2_control,
# no Gazebo.
#
#   /cmd_vel -> odrive_bridge.py -> ODrive axis0/axis1
#   ODrive encoders -> /odom + odom->base_link TF + /joint_states
#   VLP-16 -> /velodyne_packets -> /velodyne_points -> /scan
#   /scan -> slam_toolbox -> map -> odom;  /scan + /odom -> Nav2 -> /cmd_vel
#
# The Jetson must have an IPv4 address on the LiDAR's subnet or the kernel
# drops the VLP-16's UDP packets. Our sensor is at 192.168.10.201:
#     sudo ip addr add 192.168.10.100/24 dev enP8p1s0
#
# Example:
#     ros2 launch esda_simulation_2025 launch_odrive_robot.launch.py
#     ros2 launch esda_simulation_2025 launch_odrive_robot.launch.py launch_slam:=true launch_nav2:=true
#     ros2 launch esda_simulation_2025 launch_odrive_robot.launch.py launch_teleop:=false launch_joy:=true

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
    GroupAction,
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
    launch_joy = LaunchConfiguration('launch_joy')
    launch_slam = LaunchConfiguration('launch_slam')
    launch_nav2 = LaunchConfiguration('launch_nav2')
    viz_cloud_rate = LaunchConfiguration('viz_cloud_rate')
    launch_virtual_lanes = LaunchConfiguration('launch_virtual_lanes')
    lane_half_width = LaunchConfiguration('lane_half_width')
    camera_name = LaunchConfiguration('camera_name')
    launch_zed_camera = LaunchConfiguration('launch_zed_cam')
    camera_model = LaunchConfiguration('camera_model')

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
                'wheel_radius': 0.1552,
                'wheel_separation': 0.66,
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
    # ZED camera (optional)
    #
    # Scoped like SLAM/Nav2 so the wrapper's launch configurations don't leak.
    # zed_nodepth.yaml turns depth off, because the SDK install here has no
    # TensorRT and neural depth crashes the camera on open.
    # ------------------------------------------------------------

    if package_exists('zed_wrapper'):
        zed_launch = GroupAction(
            scoped=True,
            condition=IfCondition(launch_zed_camera),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('zed_wrapper'),
                            'launch', 'zed_camera.launch.py',
                        )
                    ),
                    launch_arguments={
                        'camera_name': camera_name,
                        'camera_model': camera_model,
                        # odrive_bridge.py and SLAM own odom and map; don't
                        # let the ZED node publish its own odom/map TF.
                        'publish_tf': 'false',
                        'publish_map_tf': 'false',
                        # The wrapper's launch defaults this to false, which
                        # leaves /zed/zed_node/imu/data's zed_imu_link frame
                        # with no TF.
                        'publish_imu_tf': 'true',
                        'ros_params_override_path': os.path.expanduser('~/zed_ws/zed_nodepth.yaml'),
                    }.items(),
                ),
                # Hang the wrapper's own frames off the URDF's camera_link.
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='zed_frame_link',
                    arguments=[
                        '--frame-id', 'camera_link',
                        '--child-frame-id', [camera_name, '_camera_link'],
                    ],
                    output='screen',
                ),
            ],
        )
    else:
        zed_launch = LogInfo(
            msg='zed_wrapper not found - no ZED camera. '
                'Run: source ~/zed_ws/install/setup.bash'
        )

    # Viewer-only copy of the cloud for Foxglove over Tailscale: the full
    # /velodyne_points is ~6.7 MB/s and backs up the link (odom/TF then
    # arrive in bursts). SLAM/Nav2 keep using the full-rate topics.
    cloud_viz = Node(
        package=package_name,
        executable='topic_throttle.py',
        name='velodyne_points_viz_throttle',
        output='screen',
        parameters=[{
            'input_topic': '/velodyne_points',
            'output_topic': '/velodyne_points_viz',
            'message_type': 'sensor_msgs/msg/PointCloud2',
            'rate': ParameterValue(viz_cloud_rate, value_type=float),
        }],
        condition=IfCondition(launch_lidar),
    )

    # ------------------------------------------------------------
    # SLAM Toolbox / Nav2 (optional - the GUI's SLAM and Nav2 buttons do
    # the same thing). Only resolved when enabled, so they don't need to be
    # installed to bring up the robot.
    # ------------------------------------------------------------

    #
    # Both included launch files declare `params_file`, and an include does
    # not scope its launch configurations - SLAM's mapper yaml leaked into
    # Nav2, which then fell back to its defaults (DWB, "No critics defined
    # for FollowPath") and aborted bringup. So each include gets its own
    # scoped group and an explicit params_file.

    slam = GroupAction(
        scoped=True,
        condition=IfCondition(launch_slam),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'scan_topic': '/scan',
                'params_file': os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
            }.items(),
        )],
    )

    nav2 = GroupAction(
        scoped=True,
        condition=IfCondition(launch_nav2),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'map_subscribe_transient_local': 'true',
                'scan_topic': '/scan',
                'odom_topic': '/odom',
                # Real-robot copy: larger inflation for the 0.45 m robot radius.
                'params_file': os.path.join(pkg_share, 'config', 'nav2_params_real_robot.yaml'),
            }.items(),
        )],
    )
    # Let the LiDAR, TF and SLAM's map come up before Nav2 starts.
    delayed_nav2 = TimerAction(period=8.0, actions=[nav2])

    # ------------------------------------------------------------
    # Virtual lane walls: two endless lines lane_half_width either side of
    # where the robot starts, fed to Nav2's keepout filter. Needs SLAM's
    # map frame, so it only makes sense with launch_slam.
    # ------------------------------------------------------------

    virtual_lanes = Node(
        package=package_name,
        executable='virtual_lane_walls.py',
        name='virtual_lane_walls',
        output='screen',
        parameters=[{'lane_half_width': ParameterValue(lane_half_width, value_type=float)}],
        condition=IfCondition(launch_virtual_lanes),
    )

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

    # ------------------------------------------------------------
    # Joystick teleop
    #
    # teleop_twist_joy publishes plain /cmd_vel, which is what
    # odrive_bridge.py listens on - no remap to the ros2_control topic.
    # Hold the enable button (config/joystick.yaml) to drive.
    # ------------------------------------------------------------

    joy_params = os.path.join(pkg_share, 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params],
        output='screen',
        condition=IfCondition(launch_joy),
    )

    teleop_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        output='screen',
        condition=IfCondition(launch_joy),
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
            'max_motor_turns_per_s', default_value='30.0',
            description='Motor-side speed clamp; 30 turns/s = ~0.46 m/s at 64:1'),
        DeclareLaunchArgument(
            'left_direction', default_value='-1.0',
            description='1.0 or -1.0; flip if the left wheel spins backwards'),
        DeclareLaunchArgument(
            'right_direction', default_value='1.0',
            description='1.0 or -1.0; flip if the right wheel spins backwards'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument(
            'viz_cloud_rate', default_value='2.0',
            description='Hz for /velodyne_points_viz, the throttled cloud for Foxglove'),
        DeclareLaunchArgument('launch_teleop', default_value='true'),
        DeclareLaunchArgument(
            'launch_joy', default_value='false',
            description='Run joy_node + teleop_twist_joy on /cmd_vel'),
        DeclareLaunchArgument(
            'launch_virtual_lanes', default_value='false',
            description='Two virtual walls lane_half_width either side of the start pose (needs SLAM)'),
        DeclareLaunchArgument(
            'lane_half_width', default_value='2.0',
            description='Metres from the start pose to each virtual lane wall'),
        DeclareLaunchArgument(
            'launch_slam', default_value='false',
            description='Run slam_toolbox (map -> odom) on /scan'),
        DeclareLaunchArgument(
            'launch_nav2', default_value='false',
            description='Run Nav2 on /scan + /odom (needs SLAM or AMCL for map -> odom)'),
        DeclareLaunchArgument(
            'launch_zed_cam', default_value='false',
            description='Run the ZED camera driver (needs ~/zed_ws sourced)'),
        DeclareLaunchArgument(
            'camera_model', default_value='zed2i',
            description='ZED camera model'),
        DeclareLaunchArgument(
            'camera_name', default_value='zed',
            description='ZED camera name; sets the /<name>/... topics and <name>_camera_link'),

        rsp,
        odrive_bridge,
        static_odom,

        velodyne_driver,
        velodyne_transform,
        cloud_to_scan,
        cloud_viz,

        slam,
        delayed_nav2,
        virtual_lanes,

        delayed_rviz,
        teleop,
        joy_node,
        teleop_joy,
        zed_launch,
    ])
