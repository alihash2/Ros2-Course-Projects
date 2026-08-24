cmake_minimum_required(VERSION 3.8)
project(dynamic_obstacle_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

#astarplanner
# The core library
add_library(dynamic_obstacle_avoidance_lib SHARED 
  src/astar_planner.cpp
  src/rrt_planner.cpp
  src/astar_planner_plugin.cpp
  src/rrt_planner_plugin.cpp
)
set_target_properties(dynamic_obstacle_avoidance_lib PROPERTIES OUTPUT_NAME "dynamic_obstacle_avoidance_lib")

ament_target_dependencies(dynamic_obstacle_avoidance_lib
  rclcpp
  nav_msgs
  nav2_core
  nav2_util
  nav2_costmap_2d
  pluginlib
)


target_include_directories(dynamic_obstacle_avoidance_lib PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

# The Profiling Executable
add_executable(profiling_node test/profiling_node.cpp)
target_link_libraries(profiling_node dynamic_obstacle_avoidance_lib)
ament_target_dependencies(profiling_node rclcpp nav_msgs)

# The Path Follower Node
add_executable(path_follower_node src/path_follower_node.cpp)
target_link_libraries(path_follower_node dynamic_obstacle_avoidance_lib)
ament_target_dependencies(path_follower_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
)

# The CmdVelRelay Node
add_executable(cmd_vel_relay src/cmd_vel_relay.cpp)
ament_target_dependencies(cmd_vel_relay rclcpp geometry_msgs)

install(TARGETS dynamic_obstacle_avoidance_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  profiling_node
  path_follower_node
  cmd_vel_relay
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python UI
install(PROGRAMS
  src/navigation_menu.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and params
install(DIRECTORY
  launch
  params
  plugins
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Register the plugin export
ament_export_dependencies(nav2_core)
pluginlib_export_plugin_description_file(nav2_core plugins/planner_plugin.xml)

ament_package()
