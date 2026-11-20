cmake_minimum_required(VERSION 3.8)
project(mission_uploader)

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
  src/mission_uploader.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp swarm_msgs swarm_autonomy yaml-cpp)

add_executable(mission_uploader_node src/mission_uploader_node.cpp)
ament_target_dependencies(mission_uploader_node rclcpp swarm_msgs ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} mission_uploader_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()