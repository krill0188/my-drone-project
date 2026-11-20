cmake_minimum_required(VERSION 3.8)
project(swarm_ai_perception_obstacle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(swarm_autonomy REQUIRED)
find_package(swarm_ai_perception REQUIRED)
find_package(swarm_ai_perception_model_runtime REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include
  ${CMAKE_CURRENT_SOURCE_DIR}/../common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../model_runtime/include
)

add_library(${PROJECT_NAME}
  src/obstacle_detector.cpp
  src/localization.cpp
  src/threat_assessment.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  swarm_msgs
  swarm_autonomy
  swarm_ai_perception
  swarm_ai_perception_model_runtime
  Eigen3
  OpenCV
)

# Executable for obstacle node
add_executable(obstacle_node src/obstacle_node.cpp)
ament_target_dependencies(obstacle_node
  rclcpp
  swarm_msgs
  swarm_ai_perception
  swarm_ai_perception_obstacle_detector
  sensor_msgs
  cv_bridge
  image_transport
)

install(TARGETS
  ${PROJECT_NAME}
  obstacle_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()