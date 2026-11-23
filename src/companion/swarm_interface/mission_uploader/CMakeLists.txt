cmake_minimum_required(VERSION 3.8)
project(swarm_interface_mission_uploader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(swarm_interface_intent_parser REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/mission_uploader.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  swarm_msgs
  swarm_interface_intent_parser
  nlohmann_json
  Eigen3
)

# Executable
add_executable(mission_uploader_node src/mission_uploader_node.cpp)
ament_target_dependencies(mission_uploader_node
  rclcpp
  swarm_msgs
  swarm_interface_mission_uploader
  swarm_interface_intent_parser
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  mission_uploader_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()