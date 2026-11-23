cmake_minimum_required(VERSION 3.8)
project(swarm_safety_geofence_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/geofence_monitor.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  swarm_msgs
  mavros_msgs
  Eigen3
  nlohmann_json
)

# Executable
add_executable(geofence_monitor_node src/geofence_monitor.cpp)
ament_target_dependencies(geofence_monitor_node
  rclcpp
  swarm_msgs
  mavros_msgs
  ${PROJECT_NAME}
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  geofence_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()