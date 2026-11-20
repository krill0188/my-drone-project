#!/usr/bin/env bash
# ROS 2 Humble workspace setup for companion computer

set -euo pipefail

WS_DIR="/my-drone-project/src/companion"
cd "$WS_DIR"

# Create colcon workspace structure
mkdir -p src

# Initialize rosdep if needed
if ! command -v rosdep &> /dev/null; then
    sudo apt update && sudo apt install -y python3-rosdep2
    sudo rosdep init
    rosdep update
fi

# Create package.xml template for swarm_msgs
cat > src/swarm_msgs/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>swarm_msgs</name>
  <version>1.0.0</version>
  <description>ROS 2 message definitions for AI Swarm Autonomous Drone System</description>
  <maintainer email="swarm@example.com">Swarm Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>rosidl_default_runtime</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create CMakeLists.txt for swarm_msgs
cat > src/swarm_msgs/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(swarm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgentState.msg"
  "msg/FormationState.msg"
  "msg/TaskAllocation.msg"
  "msg/AIInference.msg"
  "msg/SwarmMap.msg"
  "msg/CommQoS.msg"
  "srv/UploadMission.srv"
  "srv/RequestRTL.srv"
  "srv/EmergencyStop.srv"
  "srv/GetSwarmStatus.srv"
  "action/ExecuteMission.action"
  "action/SearchArea.action"
  "action/InspectTarget.action"
  "action/FollowMe.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

ament_package()
EOF

# Copy message definitions from contracts
cp /my-drone-project/specs/001-ai-swarm-autonomous-drones/contracts/ros2/msg/*.msg src/swarm_msgs/msg/
cp /my-drone-project/specs/001-ai-swarm-autonomous-drones/contracts/ros2/srv/*.srv src/swarm_msgs/srv/
cp /my-drone-project/specs/001-ai-swarm-autonomous-drones/contracts/ros2/action/*.action src/swarm_msgs/action/

# Create swarm_autonomy package
mkdir -p src/swarm_autonomy
cat > src/swarm_autonomy/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>swarm_autonomy</name>
  <version>1.0.0</version>
  <description>Core autonomy nodes for AI Swarm Autonomous Drone System</description>
  <maintainer email="swarm@example.com">Swarm Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>swarm_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>Eigen</depend>
  <depend>yaml-cpp</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat > src/swarm_autonomy/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(swarm_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)

# Library
add_library(${PROJECT_NAME}
  src/formation_control/formation_control.cpp
  src/task_allocator/task_allocator.cpp
  src/trajectory_planner/trajectory_planner.cpp
  src/collision_avoidance/collision_avoidance.cpp
  src/state_estimator/state_estimator.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp swarm_msgs nav_msgs tf2_ros Eigen3 yaml-cpp)

# Nodes
add_executable(formation_state_node src/formation_control/formation_state_node.cpp)
ament_target_dependencies(formation_state_node rclcpp swarm_msgs ${PROJECT_NAME})

add_executable(leader_monitor src/formation_control/leader_monitor.cpp)
ament_target_dependencies(leader_monitor rclcpp swarm_msgs ${PROJECT_NAME})

add_executable(task_allocator_node src/task_allocator/task_allocator_node.cpp)
ament_target_dependencies(task_allocator_node rclcpp swarm_msgs ${PROJECT_NAME})

add_executable(trajectory_planner_node src/trajectory_planner/trajectory_planner_node.cpp)
ament_target_dependencies(trajectory_planner_node rclcpp swarm_msgs ${PROJECT_NAME})

add_executable(collision_avoidance_node src/collision_avoidance/collision_avoidance_node.cpp)
ament_target_dependencies(collision_avoidance_node rclcpp swarm_msgs ${PROJECT_NAME})

add_executable(state_estimator_node src/state_estimator/state_estimator_node.cpp)
ament_target_dependencies(state_estimator_node rclcpp swarm_msgs ${PROJECT_NAME})

# Services
add_executable(reconfigure_service src/formation_control/reconfigure_service.cpp)
ament_target_dependencies(reconfigure_service rclcpp swarm_msgs ${PROJECT_NAME})

# Install
install(TARGETS ${PROJECT_NAME} formation_state_node leader_monitor task_allocator_node trajectory_planner_node collision_avoidance_node state_estimator_node reconfigure_service
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
EOF

# Create ai_perception package
mkdir -p src/ai_perception
cat > src/ai_perception/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ai_perception</name>
  <version>1.0.0</version>
  <description>AI perception pipeline for AI Swarm Autonomous Drone System</description>
  <maintainer email="swarm@example.com">Swarm Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>swarm_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>opencv4</depend>
  <depend>onnxruntime</depend>
  <depend>tensorrt</depend>
  <depend>yaml-cpp</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat > src/ai_perception/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(ai_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(onnxruntime REQUIRED)
find_package(TensorRT REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/model_runtime/inference_engine.cpp
  src/model_runtime/tensorrt_builder.cpp
  src/model_runtime/onnx_runner.cpp
  src/model_runtime/model_manager.cpp
  src/obstacle_detector/detector.cpp
  src/obstacle_detector/localization.cpp
  src/obstacle_detector/threat_assessment.cpp
  src/landing_assessor/classifier.cpp
  src/landing_assessor/downward_processor.cpp
  src/fault_detector/fault_classifier.cpp
  src/fault_detector/health_monitor.cpp
  src/fault_detector/severity_assessment.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp swarm_msgs sensor_msgs cv_bridge OpenCV onnxruntime TensorRT yaml-cpp)

add_executable(obstacle_node src/obstacle_detector/obstacle_node.cpp)
ament_target_dependencies(obstacle_node rclcpp swarm_msgs ${PROJECT_NAME})

add_executable(landing_node src/landing_assessor/landing_node.cpp)
ament_target_dependencies(landing_node rclcpp swarm_msgs ${PROJECT_NAME})

add_executable(fault_node src/fault_detector/fault_node.cpp)
ament_target_dependencies(fault_node rclcpp swarm_msgs ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} obstacle_node landing_node fault_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
EOF

# Create swarm_comm package
mkdir -p src/swarm_comm
cat > src/swarm_comm/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>swarm_comm</name>
  <version>1.0.0</version>
  <description>Mesh networking and consensus for AI Swarm Autonomous Drone System</description>
  <maintainer email="swarm@example.com">Swarm Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>swarm_msgs</depend>
  <depend>libsodium</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat > src/swarm_comm/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(swarm_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(libsodium REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/mesh_manager/mesh_manager.cpp
  src/consensus/consensus.cpp
  src/consensus/resync.cpp
  src/consensus/split_brain.cpp
  src/consensus/merge_protocol.cpp
  src/store_forward/store_forward.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp swarm_msgs libsodium)

add_executable(mesh_manager_node src/mesh_manager/mesh_manager_node.cpp)
ament_target_dependencies(mesh_manager_node rclcpp swarm_msgs ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} mesh_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
EOF

# Create swarm_safety package
mkdir -p src/swarm_safety
cat > src/swarm_safety/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>swarm_safety</name>
  <version>1.0.0</version>
  <description>Safety monitors for AI Swarm Autonomous Drone System</description>
  <maintainer email="swarm@example.com">Swarm Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>swarm_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat > src/swarm_safety/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(swarm_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/geofence_monitor/geofence_monitor.cpp
  src/rid_broadcaster/rid_broadcaster.cpp
  src/emergency_stop/emergency_stop.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp swarm_msgs)

add_executable(geofence_monitor_node src/geofence_monitor/geofence_monitor_node.cpp)
ament_target_dependencies(geofence_monitor_node rclcpp swarm_msgs ${PROJECT_NAME})

add_executable(rid_broadcaster_node src/rid_broadcaster/rid_broadcaster_node.cpp)
ament_target_dependencies(rid_broadcaster_node rclcpp swarm_msgs ${PROJECT_NAME})

add_executable(emergency_stop_node src/emergency_stop/emergency_stop_node.cpp)
ament_target_dependencies(emergency_stop_node rclcpp swarm_msgs ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} geofence_monitor_node rid_broadcaster_node emergency_stop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
EOF

# Create swarm_interface package
mkdir -p src/swarm_interface
cat > src/swarm_interface/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>swarm_interface</name>
  <version>1.0.0</version>
  <description>Human-swarm interface and telemetry for AI Swarm Autonomous Drone System</description>
  <maintainer email="swarm@example.com">Swarm Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>swarm_msgs</depend>
  <depend>rosbag2_cpp</depend>
  <depend>rosbag2_storage</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat > src/swarm_interface/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(swarm_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/intent_parser/intent_parser.cpp
  src/mission_uploader/mission_uploader.cpp
  src/telemetry_streamer/telemetry_streamer.cpp
  src/telemetry_streamer/rosbag_writer.cpp
  src/telemetry_streamer/log_sync.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp swarm_msgs rosbag2_cpp rosbag2_storage)

add_executable(intent_node src/intent_parser/intent_node.cpp)
ament_target_dependencies(intent_node rclcpp swarm_msgs ${PROJECT_NAME})

add_executable(mission_uploader_node src/mission_uploader/mission_uploader_node.cpp)
ament_target_dependencies(mission_uploader_node rclcpp swarm_msgs ${PROJECT_NAME})

add_executable(telemetry_streamer_node src/telemetry_streamer/telemetry_streamer_node.cpp)
ament_target_dependencies(telemetry_streamer_node rclcpp swarm_msgs ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} intent_node mission_uploader_node telemetry_streamer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
EOF

# Create swarm_common package (shared utilities)
mkdir -p src/swarm_common
cat > src/swarm_common/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>swarm_common</name>
  <version>1.0.0</version>
  <description>Common utilities for AI Swarm Autonomous Drone System</description>
  <maintainer email="swarm@example.com">Swarm Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>swarm_msgs</depend>
  <depend>yaml-cpp</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat > src/swarm_common/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(swarm_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/timestamp.cpp
  src/crypto.cpp
  src/config.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp swarm_msgs yaml-cpp)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
EOF

echo "ROS 2 workspace structure created at $WS_DIR"