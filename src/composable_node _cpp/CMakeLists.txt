cmake_minimum_required(VERSION 3.5)
project(comp_simple_publisher_subscriber_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(minimal_publisher_component_component src/publisher_member_function.cpp)

ament_export_targets(export_minimal_publisher_component)
install(TARGETS minimal_publisher_component
        EXPORT export_minimal_publisher_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

rclcpp_components_register_node(
    minimal_publisher_component
    PLUGIN "MinimalPublisher"
    EXECUTABLE minimal_publisher
)

ament_package()