cmake_minimum_required(VERSION 3.8)
project(swarm_consensus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(swarm_autonomy REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/consensus.cpp
  src/resync.cpp
  src/split_brain.cpp
  src/merge_protocol.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp swarm_msgs swarm_comm)

ament_package()