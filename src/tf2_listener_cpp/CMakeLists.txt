cmake_minimum_required(VERSION 3.5)
project(tf2_listener_cpp)

# Encontra as dependências necessárias.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
ament_target_dependencies(
    turtle_tf2_listener
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    
)

install(TARGETS
    turtle_tf2_listener
    DESTINATION lib/${PROJECT_NAME})

ament_package()
