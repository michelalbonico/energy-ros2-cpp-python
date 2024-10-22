
cmake_minimum_required(VERSION 3.8)
project(sync_async_node_cpp)

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

add_executable(SyncAsyncWriter src/sync_async_writer.cpp)
ament_target_dependencies(SyncAsyncWriter rclcpp std_msgs)
add_executable(SyncAsyncReader src/sync_async_reader.cpp)
ament_target_dependencies(SyncAsyncReader rclcpp std_msgs)

install(TARGETS
    SyncAsyncReader
    DESTINATION lib/${PROJECT_NAME})

ament_package()