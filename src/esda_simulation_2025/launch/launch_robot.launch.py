import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    package_name = "esda_simulation_2025"

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    sim_mode     = LaunchConfiguration("sim_mode", default="false")
    launch_rviz  = LaunchConfiguration("launch_rviz", default="true")

    # Path to your main xacro (adjust filename/path to your actual one)
    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        "description",
        "",
        "robot.urdf.xacro",
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " sim_mode:=", sim_mode,
            # add any other xacro args you need here
        ]),
        value_type=str
    )

    # robot_state_publisher (no IncludeLaunchDescription needed)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description,
        }],
        output="screen",
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "my_controllers_2wd.yaml"]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
            robot_controllers,
        ],
        output="screen",
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller", "--param-file", robot_controllers, "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager, on_start=[joint_broad_spawner])
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager, on_start=[diff_drive_spawner])
    )

    # RViz config
    rviz_config = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "view_bot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    # Delay RViz a bit so TF + robot_description are available
    delayed_rviz = TimerAction(period=6.0, actions=[rviz_node])

    joy_params = os.path.join(get_package_share_directory('esda_simulation_2025'), 'config', 'joystick.yaml')

    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        parameters = [joy_params],
    )

    teleop_node = Node(
        package = 'teleop_twist_joy',
        executable = 'teleop_node',
        name = 'teleop_node',
        parameters = [joy_params],
        remappings=[
            ('cmd_vel',  '/diff_drive_base_controller/cmd_vel_unstamped'),
            ('/cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("sim_mode", default_value="false"),
        rsp_node,
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,

        delayed_rviz,
        joy_node,
        teleop_node,
    ])