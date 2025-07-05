#minimum CMake Version
cmake_minimum_required(VERSION 3.2)
#project name
project(matchSimulation)
#g++ color output
SET(CMAKE_COLOR_MAKEFILE ON)

add_subdirectory("../generate_acado_solver" "acado/")

#Header directory
include_directories(
  ../common/include
  include
  ../main_robot/include
  ../embedded/include
  ../../../vision/include)

# Add all source files.
file(GLOB_RECURSE SOURCES ../common/src/*.cpp
                  ../main_robot/src/*.cpp
                  src/*.cpp)
set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CXX )

#compile flag
set(CMAKE_CXX_FLAGS "-Wall -std=c++17 -fdiagnostics-color -DSIMULATION -pthread")

# Use the package PkgConfig to detect GTK+ headers/library files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtkmm-3.0>=3.18.0)
find_package(Eigen3 REQUIRED)

unset(MIAM_UTILS_FOUND CACHE)
pkg_search_module(RPLIDAR REQUIRED rplidar)
message(STATUS "Found rplidar v." ${RPLIDAR_VERSION})
pkg_search_module(MIAM_UTILS REQUIRED miam_utils>=2.8.0)
message(STATUS "Found miam_utils v." ${MIAM_UTILS_VERSION})
include_directories(${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS} ${MIAM_UTILS_INCLUDE_DIRS})
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS})

# Include
include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})
add_definitions(${GTK3_CFLAGS_OTHER})

# Copy config.
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/config" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

# Build executables
include_directories(${CMAKE_CURRENT_BINARY_DIR}/acado/generated "../generate_acado_solver/include")
add_executable(strategyViewer ${SOURCES})
target_link_libraries(strategyViewer ${GTK3_LIBRARIES} ${RPLIDAR_LIBRARIES} ${MIAM_UTILS_LIBRARIES} acado_solver)
