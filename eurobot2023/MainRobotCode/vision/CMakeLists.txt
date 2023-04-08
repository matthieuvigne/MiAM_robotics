# CMake parameters
cmake_minimum_required(VERSION 3.2)
project(robot_vision)
set(PROJECT_DESCRIPTION "Code for the vision module")
set(CMAKE_COLOR_MAKEFILE ON)

# Building options
# Mode: Debug or RelWithDebInfo
set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
message("G++ compiler version: " ${CMAKE_CXX_COMPILER_VERSION})
add_compile_options(-std=c++17)

# Set build-type specific flags
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: debug.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -DDEBUG -I/usr/local/include")
else (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: release build - optimization enabled.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -I/usr/local/include")
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

option(RPI4 "Target is raspberry pi 4" OFF)
option(BUILD_CAMERA_CALIBRATION "Set ON to build the camera calibration script" OFF)

# Find mm libraries (useful ?)
FIND_LIBRARY(mmalcore_LIBS NAMES mmal_core PATHS /opt/vc/lib)
FIND_LIBRARY(mmalutil_LIBS NAMES mmal_util PATHS /opt/vc/lib)
FIND_LIBRARY(mmal_LIBS NAMES mmal PATHS /opt/vc/lib)
IF( (NOT mmal_LIBS ) OR (NOT mmalutil_LIBS) OR (NOT mmalcore_LIBS) )
    MESSAGE(FATAL_ERROR "Could not find mmal libraries")
ENDIF()
set(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${mmalcore_LIBS} ${mmalutil_LIBS} ${mmal_LIBS}  )

# Recursively include all the header files
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/*.hpp")
include_directories("../common/include" "include")

# Recursively include all the source files
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES
  src/*.cpp 
  ../common/src/DH_transform.cpp
  ../common/src/logger.cpp
  ../common/src/time.hpp)
set_source_files_properties(${${PROJECT_NAME}_SOURCES} PROPERTIES LANGUAGE CXX)

# Setup the static library
set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# Link to external libraries
find_package(OpenCV 4.5 REQUIRED)
message(STATUS "Found OpenCV v." ${OpenCV_VERSION})
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(MIAMUTILSLIB REQUIRED miam_utils)

message(STATUS "Found miam_utils v." ${MIAMUTILSLIB_VERSION} " at " ${MIAMUTILSLIB_LIBRARY_DIRS})
include_directories(${MIAMUTILSLIB_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_DIRS} ${Eigen_INCLUDE_DIRS} ${MIAMUTILSLIB_LIBRARY_DIRS})

if(RPI4)
    pkg_search_module(LIBCAMERA REQUIRED libcamera)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRPI4")
    include_directories(${LIBCAMERA_INCLUDE_DIRS})
    target_link_libraries(${LIBRARY_NAME} ${OpenCV_LIBS} ${LIBCAMERA_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MIAMUTILSLIB_LIBRARIES})
else()
    set(CMAKE_MODULE_PATH "/usr/local/lib/cmake/")
    find_package(raspicam REQUIRED)
    target_link_libraries(${LIBRARY_NAME} ${REQUIRED_LIBRARIES} ${OpenCV_LIBS} ${raspicam_LIBS} ${raspicam_CV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${MIAMUTILSLIB_LIBRARIES})
endif()

# Build calibration script
if(BUILD_CAMERA_CALIBRATION)
  add_executable(calibrate_camera app/calibrate_camera.cpp)
  target_link_libraries(calibrate_camera ${LIBRARY_NAME} ${Eigen_LIBRARIES})
endif()

#~ # Application
#~ add_executable(vision_module app/vision_module.cpp)
#~ target_link_libraries(vision_module ${REQUIRED_LIBRARIES} ${LIBRARY_NAME} ${Eigen_LIBRARIES} ${raspicam_LIBS} ${raspicam_CV_LIBS} stdc++fs)
#~ add_executable(camera_client app/camera_client.cpp)
#~ target_link_libraries(camera_client ${LIBRARY_NAME} ${Eigen_LIBRARIES})
