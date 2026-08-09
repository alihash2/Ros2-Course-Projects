cmake_minimum_required(VERSION 3.10)
project(mobile_robot_system)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_library(mobile_robot_lib src/mobile_robot.cpp src/wheel_data.cpp src/odometry.cpp src/lidar_data.cpp)

add_executable(mobile_robot_app apps/main.cpp)

target_link_libraries(mobile_robot_app PRIVATE mobile_robot_lib)

