import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the path to your configuration file
    # Change 'your_robot_package' to match your actual package folder name
    ekf_config_path = os.path.join(
        get_package_share_directory('esda_simulation_2025'),
        'config',
        'ekf.yaml'
    )


    # 2. Define the node execution parameters
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path] # This maps your yaml settings into the node
    )

    # 3. Return the launch system instructions
    # GPS -> odom frame, and /fromLL for converting lat/lon waypoints. Waits
    # for /gps/fix; worlds without a GPS origin simply never get one.
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('esda_simulation_2025'),
            'config', 'navsat_sim.yaml')],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/filtered'),
        ],
        condition=IfCondition(LaunchConfiguration('use_gps')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_gps', default_value='true'),
        ekf_node,
        navsat_node,
    ])
