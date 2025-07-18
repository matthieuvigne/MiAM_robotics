#minimum CMake Version
cmake_minimum_required(VERSION 3.11)
#project name
project(motionControllerTesting)

add_subdirectory("../generate_acado_solver" "acado/")

#g++ color output
SET(CMAKE_COLOR_MAKEFILE ON)
#Header directory
include_directories(
  ../common/include
  include
  ../main_robot/include
  ../../../vision/include)

option(ENABLE_TRACY "Enable tracy profiler" OFF)

option(CROSS_COMPILE "True to cross compile to arm, false to compile on current platform" OFF)

if(CROSS_COMPILE)
    set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
    set(LIBRARY_NAME ${PROJECT_NAME}_arm)
else()
    set(CMAKE_CXX_COMPILER "g++")
    set(LIBRARY_NAME ${PROJECT_NAME})
endif()

# Add all source files.
file(GLOB_RECURSE SOURCES ../common/src/*.cpp
                  src/*.cpp)
set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CXX )

#compile flag
set(CMAKE_CXX_FLAGS "-Wall -std=c++17 -fdiagnostics-color -DMOTIONCONTROLLER_UNITTEST -pthread -O3")

if (ENABLE_TRACY)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTRACY_ENABLE")
endif()

# Use the package PkgConfig to detect GTK+ headers/library files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtkmm-3.0>=3.18.0)
find_package(Eigen3 REQUIRED)

unset(MIAM_UTILS_FOUND CACHE)
unset(RPLIDAR_FOUND CACHE)
if(CROSS_COMPILE)
  pkg_search_module(RPLIDAR REQUIRED rplidar_arm)
  pkg_search_module(MIAM_UTILS REQUIRED miam_utils_arm >= 2.8.0)
else()
  pkg_search_module(RPLIDAR REQUIRED rplidar)
  pkg_search_module(MIAM_UTILS REQUIRED miam_utils>=2.7.0)
endif()

message(STATUS "Found rplidar v." ${RPLIDAR_VERSION})
message(STATUS "Found miam_utils v." ${MIAM_UTILS_VERSION})

include_directories(${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS} ${MIAM_UTILS_INCLUDE_DIRS})
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS})

# Include
include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})
add_definitions(${GTK3_CFLAGS_OTHER})


# Build solver
include_directories(${CMAKE_CURRENT_BINARY_DIR}/acado/generated "../generate_acado_solver/include")

# Copy config.
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/config" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

# Build executables
add_executable(motionControllerTesting ${SOURCES})
target_link_libraries(motionControllerTesting ${GTK3_LIBRARIES} ${RPLIDAR_LIBRARIES} ${MIAM_UTILS_LIBRARIES} acado_solver)
