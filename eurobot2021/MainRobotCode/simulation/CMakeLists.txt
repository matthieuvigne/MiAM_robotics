#minimum CMake Version
cmake_minimum_required(VERSION 3.2)
#project name
project(StrategyViewer)
#g++ color output
SET(CMAKE_COLOR_MAKEFILE ON)
#Header directory
include_directories(
  ../common/include include
  ../embedded/include
  ../../../vision/include)

# Add all source files.
file(GLOB SOURCES ../common/src/*.cpp
                  src/*.cpp
                  ../../../vision/src/network/camera_client.cpp
                  ../../../vision/src/network/client*.cpp
                  ../../../vision/src/network/socket.cpp
                  ../../../vision/src/common/maths.cpp
                  ../../../vision/src/common/logger.cpp
                  ../../../vision/src/common/marker.cpp
                  ../../../vision/src/network/server_response.cpp
                  ../../../vision/src/network/message.cpp)

#compile flag
set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -fdiagnostics-color -DSIMULATION -pthread")

# Use the package PkgConfig to detect GTK+ headers/library files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtkmm-3.0>=3.18.0)
pkg_search_module(MIAM REQUIRED miam_utils>=2.0)

# Include
include_directories(${GTK3_INCLUDE_DIRS} ${MIAM_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS} ${MIAM_LIBRARY_DIRS})
add_definitions(${GTK3_CFLAGS_OTHER} ${MIAM_CFLAGS_OTHER})

# Copy config.
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/config" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

# Build executables
add_executable(strategyViewer ${SOURCES})
target_link_libraries(strategyViewer ${GTK3_LIBRARIES} ${MIAM_LIBRARIES})
