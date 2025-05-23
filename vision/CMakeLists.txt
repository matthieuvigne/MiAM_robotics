# CMake params
cmake_minimum_required(VERSION 3.5)
project(miam_vision_arm)
set(PROJECT_DESCRIPTION "Code for the vision module")

# Building options
# Mode: Debug or RelWithDebInfo
set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
message("G++ compiler version: " ${CMAKE_CXX_COMPILER_VERSION})
add_compile_options(-std=c++17)

FIND_LIBRARY(mmalcore_LIBS NAMES mmal_core PATHS /opt/vc/lib)
FIND_LIBRARY(mmalutil_LIBS NAMES mmal_util PATHS /opt/vc/lib)
FIND_LIBRARY(mmal_LIBS NAMES mmal PATHS /opt/vc/lib)
IF( (NOT mmal_LIBS ) OR (NOT mmalutil_LIBS) OR (NOT mmalcore_LIBS) )
    MESSAGE(FATAL_ERROR "Could not find mmal libraries")
ENDIF()
set(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${mmalcore_LIBS} ${mmalutil_LIBS} ${mmal_LIBS}  )

# Set build-type specific flags
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: debug.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -DDEBUG -I/usr/local/include")
else (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: release build - optimization enabled.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -I/usr/local/include")
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

option(RPI4 "Target is raspberry pi 4" OFF)
option(USE_TEST_BENCH "Set true to simulate the vision module in a test bench" OFF)
option(BUILD_TESTS "Set ON to build the tests" OFF)
option(BUILD_CAMERA_CALIBRATION "Set ON to build the camera calibration script" OFF)

# Recursively include all the source files
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")
if(NOT USE_TEST_BENCH)
  list(REMOVE_ITEM ${PROJECT_NAME}_SOURCES "src/testing/*.cpp")
endif()
set_source_files_properties(${${PROJECT_NAME}_SOURCES} PROPERTIES LANGUAGE CXX)

# Recursively include all the header files
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/*.hpp")
if(NOT USE_TEST_BENCH)
  list(REMOVE_ITEM ${PROJECT_NAME}_HEADERS "include/testing/*.hpp")
endif()
include_directories("include")

# Setup the static library
set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# Link to external libraries
message(STATUS "Found OpenCV v." ${OpenCV_VERSION})
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(MIAMUTILSLIB REQUIRED miam_utils)
set(ENV{PKG_CONFIG_PATH} "/usr/lib/aarch64-linux-gnu/pkgconfig")
pkg_search_module(OpenCV REQUIRED opencv4 >= 4.5)

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

if(USE_TEST_BENCH)
  target_compile_definitions(${LIBRARY_NAME} PRIVATE USE_TEST_BENCH=1)
endif()

# Application

add_executable(vision_module app/vision_module.cpp)
target_link_libraries(vision_module ${REQUIRED_LIBRARIES} ${LIBRARY_NAME} ${Eigen_LIBRARIES} ${raspicam_LIBS} ${raspicam_CV_LIBS} stdc++fs)
if(USE_TEST_BENCH)
  target_compile_definitions(vision_module PRIVATE USE_TEST_BENCH=1)
endif()

add_executable(camera_client app/camera_client.cpp)
target_link_libraries(camera_client ${LIBRARY_NAME} ${Eigen_LIBRARIES})

if(BUILD_CAMERA_CALIBRATION)
  add_executable(calibrate_camera app/calibrate_camera.cpp)
  target_link_libraries(calibrate_camera ${LIBRARY_NAME} ${Eigen_LIBRARIES})
endif()

# Scripts

add_executable(test_solar_panels scripts/test_camera_solar_panels.cpp)
target_link_libraries(test_solar_panels ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

#~ add_executable(run_tag_detection scripts/run_tag_detection.cpp)
#~ target_link_libraries(run_tag_detection ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

# Tests

if(BUILD_TESTS)
  include(CTest)
  file(GLOB_RECURSE ${PROJECT_NAME}_TESTS "tests/*.cpp")
  foreach(TEST_SRC IN ITEMS ${${PROJECT_NAME}_TESTS})
    get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_link_libraries(${TEST_NAME} ${LIBRARY_NAME} ${Eigen_LIBRARIES})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
  endforeach()
endif()

# Create package config file from template.
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/miam_visionTemplate.pc" "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}.pc")

# Set install rules: copy library and headers.
install(TARGETS ${LIBRARY_NAME} DESTINATION "lib")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/" DESTINATION "include")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}.pc" DESTINATION "lib/pkgconfig/")
