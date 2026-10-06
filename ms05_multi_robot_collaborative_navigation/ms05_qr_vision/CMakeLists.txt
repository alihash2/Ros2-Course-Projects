cmake_minimum_required(VERSION 3.8)
project(ms05_qr_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QrDetection.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  scripts/qr_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME qr_detector_node
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
)

ament_package()
