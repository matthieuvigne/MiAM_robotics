cmake_minimum_required(VERSION 3.0)

#Configure project
project(vacuum_brain)
set(PROJECT_DESCRIPTION "Code for an inverted pendulum robot")

# Set compiler to arm compiler (cross-compile)
set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
#~ set(CMAKE_CXX_COMPILER "g++")

# Set C++ compiler flags.
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wno-psabi -fdiagnostics-color -pthread")

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
pkg_search_module(MIAMLIB REQUIRED miam_utils_arm>=2.4)
message(STATUS "Found miam_utils v." ${MIAMLIB_VERSION})
include_directories(${MIAMLIB_INCLUDE_DIRS})
link_directories(${MIAMLIB_LIBRARY_DIRS})

# Include source files from src directory.
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")
# Add directory containing the headers.
include_directories(include)

# Set executable target and link libraries.
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} ${MIAMLIB_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ".")

