#minimum CMake Version
cmake_minimum_required(VERSION 3.2)
#project name
project(rmdx_test)

set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
#g++ color output
SET(CMAKE_COLOR_MAKEFILE ON)

# Add all source files.
file(GLOB_RECURSE SOURCES src/*.cpp)

#compile flag
set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -fdiagnostics-color -pthread")

# Use the package PkgConfig to detect GTK+ headers/library files - cross-compiled
find_package(PkgConfig REQUIRED)

pkg_search_module(MIAM_UTILS REQUIRED miam_utils_arm >= 2.5.4)
message(STATUS "Found miam_utils v." ${MIAM_UTILS_VERSION})
include_directories(${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS} ${MIAM_UTILS_INCLUDE_DIRS})
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS})

# Build executables
add_executable(rmdx_test ${SOURCES})
target_link_libraries(rmdx_test ${GTK_LIBRARIES} ${MIAM_UTILS_LIBRARIES})
