#minimum CMake Version
cmake_minimum_required(VERSION 3.2)
#project name
project(as5045_test)

set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
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

# Find GTK+ headers/library files - cross-compiled
set(ENV{PKG_CONFIG_PATH} "/usr/lib/arm-linux-gnueabihf/pkgconfig")
pkg_search_module(HDF5LIB REQUIRED hdf5-serial)

pkg_search_module(MIAM_UTILS REQUIRED miam_utils_arm >= 2.6.1)
message(STATUS "Found miam_utils v." ${MIAM_UTILS_VERSION})
include_directories(${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS} ${MIAM_UTILS_INCLUDE_DIRS} ${HDF5LIB_INCLUDE_DIRS})
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS} ${HDF5LIB_LIBRARY_DIRS})

# Build executables
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${GTK_LIBRARIES} ${MIAM_UTILS_LIBRARIES} ${HDF5LIB_LIBRARIES} "-lhdf5_cpp")
