#minimum CMake Version
cmake_minimum_required(VERSION 3.2)
#project name
project(sts_servo_gui)

option(CROSS_COMPILE "True to cross compile to arm, false to compile on current platform" ON)

if(CROSS_COMPILE)
    set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
else()
    set(CMAKE_CXX_COMPILER "g++")
endif()

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

unset(GTK_FOUND CACHE)
unset(MIAM_UTILS_FOUND CACHE)
if(CROSS_COMPILE)
    pkg_search_module(MIAM_UTILS REQUIRED miam_utils_arm >= 2.5.4)
    set(ENV{PKG_CONFIG_PATH} "/usr/lib/aarch64-linux-gnu/pkgconfig")
else()
    pkg_search_module(MIAM_UTILS REQUIRED miam_utils >= 2.5.4)
endif()
message(STATUS "Found miam_utils v." ${MIAM_UTILS_VERSION})

pkg_search_module(GTK REQUIRED gtkmm-3.0)
include_directories(${GTK_INCLUDE_DIRS})
link_directories(${GTK_LIBRARY_DIRS})

include_directories(${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS} ${MIAM_UTILS_INCLUDE_DIRS})
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS})

# Build executables
add_executable(sts_servo_gui ${SOURCES})
target_link_libraries(sts_servo_gui ${GTK_LIBRARIES} ${MIAM_UTILS_LIBRARIES})
