cmake_minimum_required(VERSION 3.8)
project(swarm_formation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Source files
set(SOURCES
    swarm_formation.cpp
    leader_trajectory.cpp
    follower_control.cpp
    geometry_manager.cpp
    formation_fsm.cpp
    mavlink_handler.cpp
    relative_localization.cpp
    formation_slot.cpp
    follower_replan.cpp
)

# Headers
set(HEADERS
    swarm_formation.hpp
    leader_election.hpp
    follower_replan.hpp
)

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../swarm_comm
    ${CMAKE_CURRENT_SOURCE_DIR}/../swarm_safety
    ${CMAKE_CURRENT_SOURCE_DIR}/../hal
)

target_link_libraries(${PROJECT_NAME}
    swarm_comm
    swarm_safety
    swarm_hal
    mavlink
    m
)

# Install headers
install(FILES ${HEADERS}
    DESTINATION include/swarm_formation
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)