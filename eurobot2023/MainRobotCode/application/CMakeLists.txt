# CMake params
cmake_minimum_required(VERSION 3.0)
project(robot_application)
set(PROJECT_DESCRIPTION "Eurobot Appplication")

# Building options
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-std=c++11)

# Link to external libraries
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtkmm-3.0>=3.18.0)
message(STATUS "Found GTK v." ${GTK3_VERSION})

# Build the library
include_directories("include")
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/*.hpp")
set_source_files_properties(${${PROJECT_NAME}_SOURCES} PROPERTIES LANGUAGE CXX)
set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# Build the executable
include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})
add_executable(robot_app script/main.cpp)
target_link_libraries(robot_app ${LIBRARY_NAME} ${Eigen_LIBRARIES} ${GTK3_LIBRARIES})

# Generic documentation on the use of the gtkmm v3:
# https://docs.huihoo.com/gtkmm/programming-with-gtkmm-3/3.4.1/en/sec-basics-simple-example.html
# Use Glade files to build the GUI and then a builder to instantiate them fro; XML files.
# Possible to use several builders as well as multiples .glade files
