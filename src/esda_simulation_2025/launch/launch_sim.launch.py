import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():    

    # Define the input parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    use_lidar = LaunchConfiguration('use_lidar', default='true')
    world_file = LaunchConfiguration('world_file')
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='0.0')
    package_name = 'esda_simulation_2025'
    
    # ROS Controller Files:
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'config',
            'my_controllers_2wd.yaml',
        ]
    )

    # Launch robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true',
            'use_lidar': use_lidar
        }.items()
    )

    # --- 2) Launch Ignition sim server + client ---
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.dirname(get_package_share_directory(package_name)) + ':' +
            os.path.join(get_package_share_directory(package_name), 'worlds') + ':' +
            os.path.join(get_package_share_directory(package_name), 'worlds', 'models')
        ]
    )

    ign_resource_path_legacy = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.dirname(get_package_share_directory(package_name)) + ':' +
            os.path.join(get_package_share_directory(package_name), 'worlds') + ':' +
            os.path.join(get_package_share_directory(package_name), 'worlds', 'models')
        ]
    )

    ign_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py'
          )
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'verbose': 'true'
        }.items()
    )
    
    
    # --- 4) Spawn the robot_description into Ignition ---
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",  "my_robot_1",
            "-topic", "robot_description",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", "0.3",
            "-allow_renaming", "true",
            "--ros-args",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # --- 5) Load and start your controllers ---
    # Controller manager spawners
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file', robot_controllers,
            #'--ros-args', '-r', '/diff_drive_base_controller/cmd_vel:=/cmd_vel'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 6) Set up the Ros Gazebo Bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            #'/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/camera/left@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/right@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        remappings=[
            ('/camera/left', '/camera/left/image_raw'),
            ('/camera/right', '/camera/right/image_raw'),
            ('/camera/depth', '/camera/depth/image_raw'),
        ],
        output='screen'
    )
    
    # Camera frame fixer - remap Gazebo's frame_id to proper TF frame
    camera_frame_fixer = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'my_robot_1/base_link/camera'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # --- 7) Static TF publishers are handled by robot_state_publisher ---
    # The laser_frame → my_robot_1/base_link/laser_frame transform comes from URDF

    return LaunchDescription([
      DeclareLaunchArgument('use_sim_time',    default_value='true'),
      DeclareLaunchArgument('use_ros2_control',default_value='true'),
      DeclareLaunchArgument('use_lidar',       default_value='true'),
      DeclareLaunchArgument('spawn_x',         default_value='0.0'),
      DeclareLaunchArgument('spawn_y',         default_value='0.0'),
      DeclareLaunchArgument('world_file',      default_value=os.path.join(
          get_package_share_directory(package_name), 'worlds', 'sim_world_igvc.sdf')),
      rsp,
      ign_resource_path,
      ign_resource_path_legacy,
      bridge,
      camera_frame_fixer,
      ign_launch,
      spawn_entity,
      joint_broad_spawner,
      diff_drive_spawner,
    ])