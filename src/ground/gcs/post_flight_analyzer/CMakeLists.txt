cmake_minimum_required(VERSION 3.16)
project(PostFlightAnalyzerPlugin)

# QGC Plugin requirements
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find Qt6
find_package(Qt6 REQUIRED COMPONENTS Quick Core Network Positioning Qml Charts)
find_package(Qt6Qml REQUIRED)

# QGroundControl dependencies
set(QGC_SOURCE_DIR "${CMAKE_SOURCE_DIR}/../../../../../QGroundControl" CACHE PATH "QGroundControl source directory")
set(QGC_BUILD_DIR "${CMAKE_BINARY_DIR}/../../../../../QGroundControl/build" CACHE PATH "QGroundControl build directory")

# Plugin definition
set(PLUGIN_NAME "PostFlightAnalyzer")
set(PLUGIN_VERSION "1.0.0")
set(PLUGIN_JSON "${CMAKE_CURRENT_SOURCE_DIR}/PostFlightAnalyzer.json")

# Sources
set(PLUGIN_SOURCES
    PostFlightAnalyzerPlugin.cc
)

set(PLUGIN_HEADERS
    PostFlightAnalyzerPlugin.h
)

# QML files
set(QML_FILES
    qml/PostFlightAnalyzer.qml
    qml/components/CoverageHeatmapView.qml
    qml/components/FormationErrorCDFView.qml
    qml/components/AITimelineView.qml
    qml/components/QoSAnalysisView.qml
    qml/components/ReportGeneratorView.qml
)

# Python backend
set(PYTHON_FILES
    python/post_flight_backend.py
)

# Resource file for QML and Python
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/qml.qrc.in
    ${CMAKE_CURRENT_BINARY_DIR}/qml.qrc
    @ONLY
)

qt6_add_resources(PLUGIN_RESOURCES ${CMAKE_CURRENT_BINARY_DIR}/qml.qrc)

# Create plugin library
add_library(${PLUGIN_NAME} MODULE
    ${PLUGIN_SOURCES}
    ${PLUGIN_HEADERS}
    ${PLUGIN_RESOURCES}
)

target_include_directories(${PLUGIN_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PLUGIN_NAME} PRIVATE
    Qt6::Core
    Qt6::Quick
    Qt6::Network
    Qt6::Positioning
    Qt6::Qml
    Qt6::Charts
)

# Install plugin
install(TARGETS ${PLUGIN_NAME}
    LIBRARY DESTINATION ${QGC_PLUGIN_INSTALL_DIR}
    RUNTIME DESTINATION ${QGC_PLUGIN_INSTALL_DIR}
)

# Install JSON manifest
install(FILES ${PLUGIN_JSON}
    DESTINATION ${QGC_PLUGIN_INSTALL_DIR}
)

# Install QML files
install(DIRECTORY qml/
    DESTINATION ${QGC_QML_INSTALL_DIR}/PostFlightAnalyzer
    FILES_MATCHING PATTERN "*.qml"
)

# Install Python backend
install(DIRECTORY python/
    DESTINATION ${QGC_PYTHON_INSTALL_DIR}/PostFlightAnalyzer
    FILES_MATCHING PATTERN "*.py"
)

# Tests
if(BUILD_TESTING)
    enable_testing()
    find_package(Qt6Test REQUIRED)
    
    add_executable(test_post_flight_analyzer
        tests/test_coverage_analyzer.cpp
        tests/test_formation_analyzer.cpp
        tests/test_ai_analyzer.cpp
        tests/test_qos_analyzer.cpp
        tests/test_report_generator.cpp
        tests/test_python_backend.cpp
    )
    
    target_link_libraries(test_post_flight_analyzer PRIVATE
        ${PLUGIN_NAME}
        Qt6::Test
    )
    
    add_test(NAME post_flight_analyzer_tests
        COMMAND test_post_flight_analyzer
    )
endif()