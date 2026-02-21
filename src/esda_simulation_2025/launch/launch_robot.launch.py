import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
from launch_conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
# from launch.event_handlers import OnProcessExit
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Defining input parameters

    # -------------------------------
    # 1. Robot description + TF
    # -------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')
    use_lidar = LaunchConfiguration("use_lidar", default="false")
    package_name = 'esda_simulation_2025'


    sim_mode = LaunchConfiguration('sim_mode', default = 'false')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true', 'sim_mode': sim_mode}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    # controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers_2wd.yaml')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'config',
            'my_controllers_2wd.yaml',
        ]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            robot_controllers
        ],
        output="screen"
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # --- 5) Load and start your controllers ---
    # Controller manager spawners
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen'
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file', robot_controllers,
            #'--ros-args', '-r', '/diff_drive_base_controller/cmd_vel:=/cmd_vel'
        ],
        output='screen'
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    # Launching LiDar
    # velodyne_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('velodyne'),
    #             'launch',
    #             'velodyne-all-nodes-VLP16-launch.py'
    #         )
    #     ]),
    #     launch_arguments = {
    #         'params_file': os.path.join(
    #             get_package_share_directory('esda_simulation_2025'),
    #             'config',
    #             'vlp16.yaml'
    #         )
    #     }.items()
    # )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "lidar.launch.py",
            )
        ),
        condition = IfCondition(use_lidar)
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        # joystick,
        # twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])