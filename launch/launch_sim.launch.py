import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():    

    # Define the input parameters
    use_sim_time     = LaunchConfiguration('use_sim_time', default='true')
    package_name = 'esda_simulation_2025'
    
    # For Fortress/older:
    set_ign_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[get_package_share_directory('esda_simulation_2025')]
    )

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
            'use_ros2_control': 'true'
        }.items()
    )

    # --- 2) Launch Ignition sim server + client ---
    ign_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py'
          )
        ),
        launch_arguments={
            'ign_args': f'-r {os.path.join(get_package_share_directory(package_name), "worlds", "igvc.sdf")}',
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
        output='screen'
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
            '/head_camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
            '/head_camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/head_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/head_camera/images@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )
    
    # --- 7) Static TF publisher for LiDAR ---
    
    lidar_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_tf',
        arguments=['0', '0', '0.0', '0', '0', '0',  # x y z yaw pitch roll
                   '/laser_frame', 'my_robot_1/base_link/laser_frame'],
        output='screen'
    )
    
    # --- 8) Rviz spawn
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare(package_name),
            'config',
            'view_bot.rviz'
        ])],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    

    return LaunchDescription([
      DeclareLaunchArgument('use_sim_time',    default_value='true'),
      DeclareLaunchArgument('use_ros2_control',default_value='true'),
      rsp,
      set_ign_path,
      bridge,
      ign_launch,
      spawn_entity,
      joint_broad_spawner,
      diff_drive_spawner,
      lidar_transform,
      rviz,
    ])