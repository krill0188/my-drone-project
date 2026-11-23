cmake_minimum_required(VERSION 3.8)
project(swarm_interface_intent_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(swarm_interface_intent_parser REQUIRED)
find_package(nlohmann_json REQUIRED)

# TensorRT
find_package(TensorRT QUIET)
if(TensorRT_FOUND)
  message(STATUS "TensorRT found: ${TensorRT_VERSION}")
  add_compile_definitions(HAVE_TENSORRT=1)
else()
  message(STATUS "TensorRT not found")
  add_compile_definitions(HAVE_TENSORRT=0)
endif()

# ONNX Runtime
find_package(onnxruntime QUIET)
if(onnxruntime_FOUND)
  message(STATUS "ONNX Runtime found")
  add_compile_definitions(HAVE_ONNXRUNTIME=1)
else()
  add_compile_definitions(HAVE_ONNXRUNTIME=0)
endif()

# CUDA
find_package(CUDA QUIET)
if(CUDA_FOUND)
  message(STATUS "CUDA found: ${CUDA_VERSION}")
  add_compile_definitions(HAVE_CUDA=1)
else()
  add_compile_definitions(HAVE_CUDA=0)
endif()

include_directories(include)

add_library(${PROJECT_NAME}
  src/intent_parser.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  swarm_msgs
  nlohmann_json
)

if(TensorRT_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${TensorRT_LIBRARIES})
  target_include_directories(${PROJECT_NAME} PRIVATE ${TensorRT_INCLUDE_DIRS})
endif()

if(onnxruntime_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE onnxruntime)
endif()

if(CUDA_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${CUDA_LIBRARIES})
  target_include_directories(${PROJECT_NAME} PRIVATE ${CUDA_INCLUDE_DIRS})
endif()

# Executable
add_executable(intent_node src/intent_node.cpp)
ament_target_dependencies(intent_node
  rclcpp
  std_msgs
  swarm_msgs
  ${PROJECT_NAME}
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  intent_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()