cmake_minimum_required(VERSION 3.2)

project(rplidarTester)

SET(CMAKE_COLOR_MAKEFILE ON)

# Files used
include_directories(include)
file(GLOB SOURCES src/*.cpp)

# Compile flag
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -fdiagnostics-color -lpthread -pthread -lm")

# Use the package PkgConfig to detect GTK+ headers/library files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtkmm-3.0>=3.18.0)
pkg_check_modules(MIAM REQUIRED miam_utils>=2.0)
pkg_check_modules(RPLIDAR REQUIRED rplidar)

include_directories(${GTK3_INCLUDE_DIRS} ${MIAM_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS} ${MIAM_LIBRARY_DIRS} ${RPLIDAR_LIBRARY_DIRS})
add_definitions(${GTK3_CFLAGS_OTHER} ${MIAM_CFLAGS_OTHER}  ${RPLIDAR_CFLAGS_OTHER})

# Build executables
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${GTK3_LIBRARIES} ${MIAM_LIBRARIES} ${RPLIDAR_LIBRARIES})

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_SOURCE_DIR}/config/GRPLidar.glade
                ${CMAKE_CURRENT_BINARY_DIR}/GRPLidar.glade)