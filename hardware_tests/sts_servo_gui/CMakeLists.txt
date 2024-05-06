#minimum CMake Version
cmake_minimum_required(VERSION 3.2)
#project name
project(sts_servo_gui)

set(CMAKE_CXX_COMPILER "g++")
#g++ color output
SET(CMAKE_COLOR_MAKEFILE ON)
#Header directory
include_directories(include)

# Add all source files.
file(GLOB_RECURSE SOURCES src/*.cpp)

#compile flag
set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -fdiagnostics-color -pthread")

# Use the package PkgConfig to detect GTK+ headers/library files - cross-compiled
find_package(PkgConfig REQUIRED)
pkg_search_module(MIAM_UTILS REQUIRED miam_utils >= 2.5.4)
message(STATUS "Found miam_utils v." ${MIAM_UTILS_VERSION})

# set(ENV{PKG_CONFIG_PATH} "/usr/lib/arm-linux-gnueabihf/pkgconfig")
pkg_search_module(GTK REQUIRED gtkmm-3.0)
include_directories(${GTK_INCLUDE_DIRS})
link_directories(${GTK_LIBRARY_DIRS})

include_directories(${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS} ${MIAM_UTILS_INCLUDE_DIRS})
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS})

# Build executables
add_executable(sts_servo_gui ${SOURCES})
target_link_libraries(sts_servo_gui ${GTK_LIBRARIES} ${MIAM_UTILS_LIBRARIES})
