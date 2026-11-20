cmake_minimum_required(VERSION 3.8)
project(task_allocator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(swarm_autonomy REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/mission.cpp
  src/task.cpp
  src/swarm_map.cpp
  src/capability.cpp
  src/cbba.cpp
  src/auction_loop.cpp
  src/task_decomposer.cpp
  src/heterogeneous_bidding.cpp
  src/reallocation.cpp
  src/comm_handler.cpp
  src/search_patterns.cpp
  src/perimeter_surveillance.cpp
  src/distributed_sensing.cpp
  src/map_fusion.cpp
  src/semantic_fusion.cpp
  src/nofly_sync.cpp
  src/landmark_correction.cpp
  src/capability_advertisement.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp swarm_msgs swarm_autonomy yaml-cpp GTSAM Eigen3)

ament_package()