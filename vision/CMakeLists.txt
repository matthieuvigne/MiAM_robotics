# CMake params
cmake_minimum_required(VERSION 2.8.3)
project(vision)
set(PROJECT_DESCRIPTION "Code for the vision module")

# Building options
# Mode: Debug or RelWithDebInfo
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-std=c++11)

# Recursively include all the source files
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")
set_source_files_properties(${${PROJECT_NAME}_SOURCES} PROPERTIES LANGUAGE CXX)

# Recursively include all the header files
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/*.hpp")
include_directories("include")

# Setup the static library
set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# Link to external libraries
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(Threads REQUIRED)
find_package(raspicam REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(MIAMUTILSLIB REQUIRED miam_utils)
message(STATUS "Found miam_utils v." ${MIAMUTILSLIB_VERSION})
include_directories(${MIAMUTILSLIB_INCLUDE_DIRS})
link_directories(${OpenCV_DIRS} ${Eigen_INCLUDE_DIRS} ${MIAMUTILSLIB_LIBRARY_DIRS})
target_link_libraries(${LIBRARY_NAME} ${OpenCV_LIBS} yaml-cpp raspicam ${CMAKE_THREAD_LIBS_INIT} ${MIAMUTILSLIB_LIBRARIES})

# Application

add_executable(vision_module app/vision_module.cpp)
target_link_libraries(vision_module ${LIBRARY_NAME} ${Eigen_LIBRARIES})

add_executable(dummy_client app/dummy_client.cpp)
target_link_libraries(dummy_client ${LIBRARY_NAME} ${Eigen_LIBRARIES})
