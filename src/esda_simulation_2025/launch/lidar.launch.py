import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory("esda_simulation_2025")
    params = os.path.join(pkg, "config", "vlp16.yaml")

    driver = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        output="screen",
        parameters=[params],
    )

    transform = Node(
        package="velodyne_pointcloud",
        executable="velodyne_transform_node",
        name="velodyne_transform_node",
        output="screen",
        parameters=[params],
    )

    laserscan = Node(
        package="velodyne_laserscan",
        executable="velodyne_laserscan_node",
        name="velodyne_laserscan_node",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([driver, transform, laserscan])