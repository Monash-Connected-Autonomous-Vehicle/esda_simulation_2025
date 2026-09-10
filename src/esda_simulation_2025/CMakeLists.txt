cmake_minimum_required(VERSION 3.5)
project(esda_simulation_2025)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  description
  config
  launch
  worlds
  assets
  maps
  behaviour_tree
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/lane_detection.py
  scripts/lane_detection_FCN.py
  scripts/lane_detection_twinlite.py
  scripts/teleop_wasd.py
  scripts/ui_launch.py
  scripts/waypoint_navigator.py
  scripts/follow_the_gap.py
  scripts/track_follower.py
  scripts/behaviour_tree.py
  scripts/waypoint_navigator_recommendation.py
  scripts/wavefront_frontier_exploration.py
  scripts/curve_detection.py
  scripts/throttle_publisher.py
  scripts/cmd_vel_odometry.py
  scripts/simple_lane_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigationRecommendation.msg"
  "msg/LaneParameters.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
