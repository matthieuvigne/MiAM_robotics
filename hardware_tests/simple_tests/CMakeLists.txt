#minimum CMake Version
cmake_minimum_required(VERSION 3.5)
#project name
project(simple_tests)

set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
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

pkg_search_module(MIAM_UTILS REQUIRED miam_utils_arm >= 2.6.1)
message(STATUS "Found miam_utils v." ${MIAM_UTILS_VERSION})


include_directories(${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS} ${MIAM_UTILS_INCLUDE_DIRS})
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS})

# Build executables
add_executable(test_as5045 TestAS5045.cpp)
target_link_libraries(test_as5045 ${GTK_LIBRARIES} ${MIAM_UTILS_LIBRARIES})


add_executable(test_ina226 TestINA226.cpp)
target_link_libraries(test_ina226 ${GTK_LIBRARIES} ${MIAM_UTILS_LIBRARIES})

add_executable(test_mcp23008 TestMCP23008.cpp)
target_link_libraries(test_mcp23008 ${GTK_LIBRARIES} ${MIAM_UTILS_LIBRARIES})

add_executable(test_as5048B TestAS5048B.cpp)
target_link_libraries(test_as5048B ${GTK_LIBRARIES} ${MIAM_UTILS_LIBRARIES})

add_executable(test_logger TestLogger.cpp)
target_link_libraries(test_logger ${GTK_LIBRARIES} ${MIAM_UTILS_LIBRARIES})

# Vision test requires opencv
set(ENV{PKG_CONFIG_PATH} "/usr/lib/aarch64-linux-gnu/pkgconfig")
pkg_search_module(OpenCV REQUIRED opencv4 >= 4.5)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(../../vision/include)
add_executable(test_camera TestSolarPanelCamera.cpp ../../vision/src/common/solar_panel_camera.cpp)
target_link_libraries(test_camera ${GTK_LIBRARIES} ${MIAM_UTILS_LIBRARIES} ${OpenCV_LIBRARIES})
