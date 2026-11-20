cmake_minimum_required(VERSION 3.8)
project(trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(swarm_autonomy REQUIRED)
find_package(swarm_ai_perception REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(include
  ${CMAKE_CURRENT_SOURCE_DIR}/../state_estimator/include
)

# Library
add_library(${PROJECT_NAME}
  src/rrt_star.cpp
  src/mpc_planner.cpp
  src/trajectory_planner.cpp
  src/formation_trajectory_planner.cpp
  src/hybrid_planner.cpp
  src/ai_replan_integration.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp swarm_msgs swarm_autonomy swarm_ai_perception Eigen3 GTSAM)

# Nodes
add_executable(trajectory_planner_node src/trajectory_planner_node.cpp)
ament_target_dependencies(trajectory_planner_node rclcpp swarm_msgs ${PROJECT_NAME})

# Install
install(TARGETS ${PROJECT_NAME} trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()