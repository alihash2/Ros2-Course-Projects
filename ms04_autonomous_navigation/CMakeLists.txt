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
find_package(visualization_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
set(msg_files
  "msg/NavigationMission.msg"
  "msg/NavigationEvent.msg"
  "msg/NavigationStatus.msg"
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

# Build navigation coordinator node
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(navigation_coordinator_node
  src/navigation_coordinator_node.cpp
)
target_include_directories(navigation_coordinator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(navigation_coordinator_node
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  builtin_interfaces
  action_msgs
  visualization_msgs
)
target_link_libraries(navigation_coordinator_node ${cpp_typesupport_target})

install(TARGETS
  cmd_vel_relay
  navigation_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/auto_slam_explorer.py
  scripts/mission_control_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories for launch, worlds, models, maps, params, rviz
install(DIRECTORY
  launch
  worlds
  models
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
  visualization_msgs
  action_msgs
  builtin_interfaces
)

ament_package()
