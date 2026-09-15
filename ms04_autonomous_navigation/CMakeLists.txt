cmake_minimum_required(VERSION 3.8)
project(ms04_autonomous_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
set(msg_files
  "msg/NavigationMission.msg"
  "msg/NavigationEvent.msg"
)

set(action_files
  "action/ExecuteMission.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
    action_msgs
)

# Build cmd_vel_relay node
add_executable(cmd_vel_relay src/cmd_vel_relay.cpp)
ament_target_dependencies(cmd_vel_relay
  rclcpp
  geometry_msgs
)

install(TARGETS
  cmd_vel_relay
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/auto_slam_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories for launch, worlds, maps, params, rviz
install(DIRECTORY
  launch
  worlds
  maps
  params
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclcpp_action
  rclpy
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  action_msgs
  builtin_interfaces
)

ament_package()
