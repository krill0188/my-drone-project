cmake_minimum_required(VERSION 3.8)
project(swarm_safety_rid_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/rid_monitor.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  swarm_msgs
  mavros_msgs
  Eigen3
  nlohmann_json
  CURL::libcurl
)

# Executable
add_executable(rid_monitor_node src/rid_monitor.cpp)
ament_target_dependencies(rid_monitor_node
  rclcpp
  swarm_msgs
  mavros_msgs
  ${PROJECT_NAME}
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  rid_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()