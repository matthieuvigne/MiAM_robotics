cmake_minimum_required(VERSION 3.5)

#Configure project
project(mainRobotCode)
set(PROJECT_DESCRIPTION "Code for the main robot for Eurobot")

set(NO_ACADO ON)
add_subdirectory("../generate_acado_solver" "acado/")

# Set compiler to arm compiler (cross-compile)
set(CMAKE_CXX_COMPILER "/usr/bin/aarch64-linux-gnu-g++")

# Set C++ compiler flags.
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wno-psabi -fdiagnostics-color -pthread")

# Set build-type specific flags
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: debug.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -DDEBUG")
else (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: release build - optimization enabled.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

# Use pkg-config to find rplidar
find_package(PkgConfig REQUIRED)
pkg_search_module(RPLIDAR REQUIRED rplidar_arm)
message(STATUS "Found rplidar_arm v." ${RPLIDAR_VERSION})
unset(MIAM_UTILS_FOUND CACHE)
pkg_search_module(MIAM_UTILS REQUIRED miam_utils_arm >= 2.8.0)
message(STATUS "Found miam_utils_arm v." ${MIAM_UTILS_VERSION})

set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib)

# Find opencv
# find_package(OpenCV REQUIRED)
# message(STATUS "Found OpenCV v." ${OpenCV_VERSION})
# set(OpenCV_INCLUDE_DIRS "/usr/local/include/opencv4")
# set(OpenCV_LIBRARY_DIRS "/usr/local/lib")

# Find GTK+ headers/library files - cross-compiled
set(ENV{PKG_CONFIG_PATH} "/usr/lib/aarch64-linux-gnu/pkgconfig")
pkg_search_module(GTK REQUIRED gtkmm-3.0)

find_package(Eigen3 REQUIRED)
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${RPLIDAR_INCLUDE_DIRS}
  ${MIAM_UTILS_INCLUDE_DIRS}
  ${GTK_INCLUDE_DIRS}
  )
link_directories(
  ${RPLIDAR_LIBRARY_DIRS}
  ${MIAM_UTILS_LIBRARY_DIRS}
  ${GTK_LIBRARY_DIRS}
  )

# Include source files from src directory.
file(GLOB_RECURSE COMMON_SOURCES "../common/src/*.cpp"
                                 "src/Robot.cpp")

file(GLOB_RECURSE MAIN_SOURCES "../main_robot/src/*.cpp" "src/MainRobotMain.cpp")

set_source_files_properties(${${PROJECT_NAME}_SOURCES} PROPERTIES LANGUAGE CXX )

# Add directory containing the headers.
include_directories(../common/include)
include_directories(../../../tracy/public)
include_directories(../main_robot/include)
include_directories(./include)


# Need to cross-compile acado solver
set(ACADO_GEN_FILES acado/generated/acado_auxiliary_functions.c
			              acado/generated/acado_common.h
			              acado/generated/acado_qpoases_interface.cpp
			              acado/generated/acado_solver.c
			              acado/generated/acado_auxiliary_functions.h
			              acado/generated/acado_integrator.c
			              acado/generated/acado_qpoases_interface.hpp)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/acado/generated
                    ../../qpoases
                    ../../qpoases/INCLUDE
                    ../../qpoases/SRC)
FILE(GLOB QPOASES_SOURCES ../../qpoases/SRC/*.cpp ../../qpoases/SRC/*.hpp )
set_source_files_properties(${ACADO_GEN_FILES} PROPERTIES LANGUAGE CXX )

ADD_LIBRARY(acado_solver_arm ${ACADO_GEN_FILES} ${QPOASES_SOURCES})
add_dependencies(acado_solver_arm acado_codegen)
include_directories("../generate_acado_solver/include")

# Set executable target and link libraries.
add_executable(mainRobotCode ${MAIN_SOURCES} ${COMMON_SOURCES})
target_link_libraries(mainRobotCode
  ${MIAM_UTILS_LIBRARIES}
  ${RPLIDAR_LIBRARIES}
  ${GTK_LIBRARIES}
  acado_solver_arm)

install(TARGETS ${PROJECT_NAME} DESTINATION ".")
