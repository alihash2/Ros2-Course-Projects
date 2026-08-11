cmake_minimum_required(VERSION 3.10)
project(ms02_mobile_robot_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ros2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

#export include dir
include_directories(include)

add_library(goto_pose_node_lib src/goto_pose_node.cpp)
ament_target_dependencies(goto_pose_node_lib 
rclcpp 
geometry_msgs 
turtlesim
)

ament_package()
