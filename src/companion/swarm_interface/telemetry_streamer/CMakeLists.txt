cmake_minimum_required(VERSION 3.8)
project(swarm_interface_telemetry_streamer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(swarm_interface_telemetry_streamer REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/telemetry_streamer.cpp
  src/rosbag_writer.cpp
  src/log_sync.cpp
  src/log_analyzer.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  swarm_msgs
  swarm_interface_telemetry_streamer
  rosbag2_cpp
  rosbag2_storage
  Eigen3
)

# Executable
add_executable(telemetry_streamer_node src/telemetry_streamer_node.cpp)
ament_target_dependencies(telemetry_streamer_node
  rclcpp
  swarm_msgs
  swarm_interface_telemetry_streamer
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  telemetry_streamer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()