cmake_minimum_required(VERSION 3.8)
project(ms05_auto_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ms05_qr_vision REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QrTagRecord.msg"
  "srv/RecordQrTag.srv"
  "srv/QueryQrTag.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  scripts/qr_tag_database_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME qr_tag_database_node
)

install(DIRECTORY
  launch
  params
  config
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
)

ament_package()
