# CMake params
cmake_minimum_required(VERSION 3.0)
project(vision)
set(PROJECT_DESCRIPTION "Code for the vision module")

# Building options
# Mode: Debug or RelWithDebInfo
set(CMAKE_CXX_COMPILER "g++-10")
message("G++ compiler version: " ${CMAKE_CXX_COMPILER_VERSION})
add_compile_options(-std=c++17)

# Set build-type specific flags
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: debug.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -DDEBUG")
else (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: release build - optimization enabled.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

option(RPI4 "Target is raspberry pi 4" OFF)
option(USE_TEST_BENCH "Set true to simulate the vision module in a test bench" ON)
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
find_package(OpenCV 4.5 REQUIRED)
message(STATUS "Found OpenCV v." ${OpenCV_VERSION})
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(YAMLCPP REQUIRED yaml-cpp)
pkg_search_module(MIAMUTILSLIB REQUIRED miam_utils)

message(STATUS "Found miam_utils v." ${MIAMUTILSLIB_VERSION} " at " ${MIAMUTILSLIB_LIBRARY_DIRS})
include_directories(${MIAMUTILSLIB_INCLUDE_DIRS})
link_directories(${OpenCV_DIRS} ${Eigen_INCLUDE_DIRS} ${YAMLCPP_INCLUDE_DIRS} ${MIAMUTILSLIB_LIBRARY_DIRS})

if(RPI4)
    pkg_search_module(LIBCAMERA REQUIRED libcamera)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRPI4")
    include_directories(${LIBCAMERA_INCLUDE_DIRS})
    target_link_libraries(${LIBRARY_NAME} ${OpenCV_LIBS} ${LIBCAMERA_LIBRARIES} ${YAMLCPP_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MIAMUTILSLIB_LIBRARIES})
else()
    find_package(raspicam REQUIRED)
    target_link_libraries(${LIBRARY_NAME} ${OpenCV_LIBS} ${raspicam_CV_LIBS} ${YAMLCPP_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MIAMUTILSLIB_LIBRARIES})
endif()

if(USE_TEST_BENCH)
  target_compile_definitions(${LIBRARY_NAME} PRIVATE USE_TEST_BENCH=1)
endif()

# Application

add_executable(vision_module app/vision_module.cpp)
target_link_libraries(vision_module ${LIBRARY_NAME} ${Eigen_LIBRARIES} stdc++fs)
if(USE_TEST_BENCH)
  target_compile_definitions(vision_module PRIVATE USE_TEST_BENCH=1)
endif()

add_executable(dummy_client app/dummy_client.cpp)
target_link_libraries(dummy_client ${LIBRARY_NAME} ${Eigen_LIBRARIES})

# add_executable(testapp app/test.cpp)
# target_link_libraries(testapp ${LIBRARY_NAME})

if(BUILD_CAMERA_CALIBRATION)
  add_executable(calibrate_camera app/calibrate_camera.cpp)
  target_link_libraries(calibrate_camera ${LIBRARY_NAME} ${Eigen_LIBRARIES})
endif()

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

# Set install rules: copy library and headers.
#~ install(TARGETS ${LIBRARY_NAME} DESTINATION "lib")
#~ install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/" DESTINATION "include")
#~ install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}.pc" DESTINATION "lib/pkgconfig/")
