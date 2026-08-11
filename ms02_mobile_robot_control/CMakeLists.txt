cmake_minimum_required(VERSION 3.10)
project(ms02_mobile_robot_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(include)

add_library(goto_pose_node_lib src/goto_pose_node.cpp)

ament_target_dependencies(goto_pose_node_lib 
rclcpp 
geometry_msgs 
turtlesim
)

add_executable(goto_pose_app apps/goto_pose_node_app.cpp)

target_link_libraries(goto_pose_app goto_pose_node_lib)

ament_target_dependencies(goto_pose_app
rclcpp
geometry_msgs
turtlesim
)

install(TARGETS
    goto_pose_app
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
