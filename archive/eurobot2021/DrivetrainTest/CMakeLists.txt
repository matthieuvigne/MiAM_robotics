cmake_minimum_required(VERSION 3.0)

#Configure project
project(drivetrainTest)
set(PROJECT_DESCRIPTION "Test code for 2019 drivetrain")

# Set compiler to arm compiler (cross-compile)
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS_DEBUG "-DDEBUG")

# Set C compiler flags.
set(CMAKE_CXX_FLAGS "-Wall -fdiagnostics-color -lpthread")

# Use pkg-config to find external libraries : Glib (compiled for arm) and BBBEurobot.
find_package(PkgConfig REQUIRED)
# Look for external libraries, and link them to the project.
unset(ARMGLIB_FOUND CACHE)
unset(MIAMLIB_FOUND CACHE)
pkg_search_module(MIAMLIB REQUIRED miam_utils>=2.0)
message(STATUS "Found MiAMEurobot v." ${MIAMLIB_VERSION})

include_directories(${ARMGLIB_INCLUDE_DIRS} ${MIAMLIB_INCLUDE_DIRS})
link_directories(${ARMGLIB_LIBRARY_DIRS} ${MIAMLIB_LIBRARY_DIRS})

# Include source files from src directory.
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.c" "src/*.cpp")
# Add directory containing the headers.
include_directories(include)

set_source_files_properties(${${PROJECT_NAME}_SOURCES} PROPERTIES LANGUAGE CXX )


# Set executable target and link libraries.
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} ${ARMGLIB_LIBRARIES} ${MIAMLIB_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ".")

