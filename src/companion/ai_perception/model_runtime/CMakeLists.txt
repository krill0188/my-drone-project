cmake_minimum_required(VERSION 3.8)
project(swarm_ai_perception_model_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(swarm_autonomy REQUIRED)
find_package(swarm_ai_perception REQUIRED)
find_package(Eigen3 REQUIRED)

# TensorRT
find_package(TensorRT QUIET)
if(TensorRT_FOUND)
  message(STATUS "TensorRT found: ${TensorRT_VERSION}")
  add_compile_definitions(HAVE_TENSORRT=1)
else()
  message(STATUS "TensorRT not found, using stub implementation")
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

# OpenCV (for calibration)
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
  message(STATUS "OpenCV found: ${OpenCV_VERSION}")
  add_compile_definitions(HAVE_OPENCV=1)
else()
  message(STATUS "OpenCV not found, calibration may not work")
  add_compile_definitions(HAVE_OPENCV=0)
endif()

include_directories(include
  ${CMAKE_CURRENT_SOURCE_DIR}/../common/include
)

add_library(${PROJECT_NAME}
  src/tensorrt_builder.cpp
  src/onnx_runner.cpp
  src/inference_engine.cpp
  src/model_manager.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  swarm_msgs
  swarm_autonomy
  swarm_ai_perception
  Eigen3
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

if(OpenCV_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
  target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
endif()

ament_package()