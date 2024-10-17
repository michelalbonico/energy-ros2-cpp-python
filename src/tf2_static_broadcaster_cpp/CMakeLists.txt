cmake_minimum_required(VERSION 3.5)
project(tf2_static_broadcaster_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)

target_include_directories(static_turtle_tf2_broadcaster PUBLIC
  $<BUILD_INTERFACE:${tf2_ros_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
   static_turtle_tf2_broadcaster
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)
install(TARGETS
   static_turtle_tf2_broadcaster
   DESTINATION lib/${PROJECT_NAME})

ament_package()
