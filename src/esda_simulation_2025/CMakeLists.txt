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
  scripts/lane_detection_esda.py
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
  scripts/esp32_wheel_bridge.py
  scripts/odrive_bridge.py
  scripts/odrive_monitor.py
  scripts/topic_throttle.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# The YOLO lane+cone segmentation node.
#
# It is installed FROM research/vision/ rather than copied into scripts/.
# Two copies of a detector drift, and this project has already lost a day to
# the wrong copy of a workspace being the one that ran. research/vision is
# also where its training and scoring tools live, and they import the same
# two modules it does.
#
# Guarded, so a checkout of this package on its own still configures -- it
# just does not get the node, and `ros2 run` then says so, which is true.
# ---------------------------------------------------------------------------
set(ESDA_VISION_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../research/vision)
set(ESDA_VISION_MODELS ${CMAKE_CURRENT_SOURCE_DIR}/../../research/models)
if(EXISTS ${ESDA_VISION_DIR}/lane_seg_node.py)
  install(PROGRAMS
    ${ESDA_VISION_DIR}/lane_seg_node.py
    ${ESDA_VISION_DIR}/lane_link.py
    ${ESDA_VISION_DIR}/lane_classic.py
    DESTINATION lib/${PROJECT_NAME}
  )
  # share/<pkg>/vision/vision_config.yaml and share/<pkg>/models/<weight>:
  # the config's relative "../models/..." then resolves the same way
  # installed as it does in the source tree.
  install(FILES ${ESDA_VISION_DIR}/vision_config.yaml
    DESTINATION share/${PROJECT_NAME}/vision
  )
  if(EXISTS ${ESDA_VISION_MODELS}/lane_seg_synth_cpu.pt)
    install(FILES ${ESDA_VISION_MODELS}/lane_seg_synth_cpu.pt
      DESTINATION share/${PROJECT_NAME}/models
    )
  else()
    message(WARNING
      "research/models/lane_seg_synth_cpu.pt is missing; lane_seg_node.py "
      "will only work with --detector classic")
  endif()
else()
  message(WARNING
    "research/vision/ not found; the lane+cone node will not be installed")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigationRecommendation.msg"
  "msg/LaneParameters.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
