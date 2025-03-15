#minimum CMake Version
cmake_minimum_required(VERSION 3.2)
#project name
project(nautilus_test)

set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
#g++ color output
SET(CMAKE_COLOR_MAKEFILE ON)


#compile flag
set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -fdiagnostics-color -pthread")

# Use the package PkgConfig to detect GTK+ headers/library files - cross-compiled
find_package(PkgConfig REQUIRED)

set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib)
pkg_search_module(MIAM_UTILS REQUIRED miam_utils_arm >= 2.5.4)

message(STATUS "Found miam_utils v." ${MIAM_UTILS_VERSION})
include_directories(${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS} ${MIAM_UTILS_INCLUDE_DIRS})
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS})

# Add all source files.
file(GLOB_RECURSE SOURCES src/*.cpp)

# Build executables
add_executable(nautilus_test ${SOURCES})
target_link_libraries(nautilus_test ${GTK_LIBRARIES} ${MIAM_UTILS_LIBRARIES})
