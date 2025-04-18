cmake_minimum_required(VERSION 3.0)

#Configure project
project(mainRobotCode)
set(PROJECT_DESCRIPTION "Code for the main robot for Eurobot")

# Set compiler to arm compiler (cross-compile)
set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")

# Set C++ compiler flags.
set(CMAKE_CXX_FLAGS "-Wall -Wno-psabi -fdiagnostics-color -pthread")

# Set build-type specific flags
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: debug.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -DDEBUG")
else (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: release build - optimization enabled.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

# Use pkg-config to find external libraries : Glib (compiled for arm) and BBBEurobot.
find_package(PkgConfig REQUIRED)
# Look for external libraries, and link them to the project.
unset(MIAMLIB_FOUND CACHE)
pkg_search_module(MIAMLIB REQUIRED miam_utils_arm>=2.5.2)
message(STATUS "Found MiAMEurobot v." ${MIAMLIB_VERSION})
pkg_search_module(RPLIDAR REQUIRED rplidar_arm)

# Eigen - for camera
find_package(Eigen3 REQUIRED)

include_directories(${MIAMLIB_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS})
link_directories(${MIAMLIB_LIBRARY_DIRS} ${RPLIDAR_LIBRARY_DIRS})

# Include source files from src directory.
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "../common/src/*.cpp"
                                          "src/*.cpp"
                                          "../../../vision/src/network/camera_client.cpp"
                                          "../../../vision/src/network/client_request.cpp"
                                          "../../../vision/src/network/client_socket.cpp"
                                          "../../../vision/src/network/socket.cpp"
                                          "../../../vision/src/common/maths.cpp"
                                          "../../../vision/src/common/logger.cpp"
                                          "../../../vision/src/common/marker.cpp"
                                          "../../../vision/src/network/server_response.cpp"
                                          "../../../vision/src/network/message.cpp")
                                          
# Add directory containing the headers.
include_directories(../common/include)
include_directories(./include)
include_directories(../../../vision/include)

# Set executable target and link libraries.
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} ${MIAMLIB_LIBRARIES} ${RPLIDAR_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ".")

