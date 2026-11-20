cmake_minimum_required(VERSION 3.8)
project(collision_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(swarm_autonomy REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/orca.cpp
  src/mpc_avoidance.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp swarm_msgs swarm_autonomy Eigen3 GTSAM)

ament_package()