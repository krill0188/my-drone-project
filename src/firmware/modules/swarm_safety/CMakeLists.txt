cmake_minimum_required(VERSION 3.8)
project(swarm_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20 -fno-exceptions -fno-rtti)
endif()

# DAL A compliance: strict warnings as errors
add_compile_options(-Werror=return-type -Werror=uninitialized -Werror=switch)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(swarm_comm REQUIRED)
find_package(swarm_hal REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# Library
add_library(${PROJECT_NAME}
  src/swarm_safety.cpp
  src/safety_events.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  swarm_msgs
  swarm_comm
  swarm_hal
  Eigen3
)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  add_executable(test_geofence
    tests/test_geofence.cpp
  )
  ament_target_dependencies(test_geofence ${PROJECT_NAME} Eigen3)
  
  add_executable(test_emergency_stop
    tests/test_emergency_stop.cpp
  )
  ament_target_dependencies(test_emergency_stop ${PROJECT_NAME})
  
  add_executable(test_rid_broadcaster
    tests/test_rid_broadcaster.cpp
  )
  ament_target_dependencies(test_rid_broadcaster ${PROJECT_NAME})
endif()

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()