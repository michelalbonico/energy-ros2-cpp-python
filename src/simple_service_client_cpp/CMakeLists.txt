cmake_minimum_required(VERSION 3.5)
project(simple_service_client_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(service_server src/add_two_ints_server.cpp)
ament_target_dependencies(service_server rclcpp example_interfaces)

add_executable(service_client src/add_two_ints_client.cpp)
ament_target_dependencies(service_client rclcpp example_interfaces)

install(TARGETS
  service_server
  service_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()