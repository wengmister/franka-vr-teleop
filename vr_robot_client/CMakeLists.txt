cmake_minimum_required(VERSION 3.10)
project(franka_vr_control_client)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set CMAKE_PREFIX_PATH to find your libfranka installation
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};$ENV{HOME}/ws/franka/install")

# Find required packages
find_package(Franka REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# Add examples_common as a library
add_library(examples_common src/examples_common.cpp)
target_link_libraries(examples_common Franka::Franka)
target_include_directories(examples_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR})

# Add VR Franka control executable
add_executable(franka_vr_control_client src/franka_vr_control_client.cpp)

# Link libraries
target_link_libraries(franka_vr_control_client
    Franka::Franka
    Threads::Threads
    examples_common
    ${EIGEN3_LIBRARIES}
)

# Include directories
target_include_directories(franka_vr_control_client PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR} 
    ${EIGEN3_INCLUDE_DIR}
)

# Optional: Set runtime path for shared libraries
set_target_properties(franka_vr_control_client PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
)