cmake_minimum_required(VERSION 3.0)

#Configure project
project(sts_servo_test)
set(PROJECT_DESCRIPTION "Test for the servos")

# Set compiler to arm compiler (cross-compile)
set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")

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
pkg_search_module(MIAM_UTILS REQUIRED miam_utils_arm >= 2.7.0)
message(STATUS "Found miam_utils_arm v." ${MIAM_UTILS_VERSION})
unset(MIAM_VISION_FOUND CACHE)
pkg_search_module(MIAM_VISION REQUIRED miam_vision_arm)
message(STATUS "Found miam_vision_arm v." ${MIAM_VISION_VERSION})

set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib)

# Find GTK+ headers/library files - cross-compiled
set(ENV{PKG_CONFIG_PATH} "/usr/lib/arm-linux-gnueabihf/pkgconfig")
pkg_search_module(GTK REQUIRED gtkmm-3.0)
pkg_search_module(HDF5LIB REQUIRED hdf5-serial)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS} ${MIAM_UTILS_INCLUDE_DIRS} ${MIAM_VISION_INCLUDE_DIRS} ${GTK_INCLUDE_DIRS} ${HDF5LIB_INCLUDE_DIRS})
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS} ${MIAM_VISION_LIBRARY_DIRS} ${GTK_LIBRARY_DIRS} ${HDF5LIB_LIBRARY_DIRS})

# Include source files from src directory.
file(GLOB_RECURSE COMMON_SOURCES "../../eurobot2024/MainRobotCode/common/src/*.cpp"
                                 "../../eurobot2024/MainRobotCode/embedded/src/Robot.cpp")

file(GLOB_RECURSE MAIN_SOURCES "../../eurobot2024/MainRobotCode/main_robot/src/*.cpp"
                               "./src/sts_servo_test.cpp")

set_source_files_properties(${${PROJECT_NAME}_SOURCES} PROPERTIES LANGUAGE CXX )

# Add directory containing the headers.
include_directories(../../eurobot2024/MainRobotCode/common/include)
include_directories(../../eurobot2024/MainRobotCode/main_robot/include)
include_directories(../../eurobot2024/MainRobotCode/embedded/include)
include_directories(./include)
include_directories(../../a-star/source
    ../../qpoases
    ../../qpoases/INCLUDE
    ../../qpoases/SRC
    ../../eurobot2024/MainRobotCode/common/acado_solver)

FILE( GLOB LIB_SOURCES ../../a-star/source/*.cpp )
FILE( GLOB LIB_HEADERS ../../a-star/source/*.hpp )
set_source_files_properties(${LIB_SOURCES} PROPERTIES LANGUAGE CXX )
ADD_LIBRARY( AStar ${LIB_SOURCES} ${LIB_HEADERS} )

# qpoases
FILE( GLOB LIB_SOURCES ../../qpoases/SRC/*.cpp )
FILE( GLOB LIB_HEADERS ../../qpoases/INCLUDE/*.hpp )
set_source_files_properties(${LIB_SOURCES1} PROPERTIES LANGUAGE CXX )
ADD_LIBRARY( qpoases ${LIB_SOURCES} ${LIB_HEADERS} )

# acado_solver
FILE( GLOB LIB_SOURCES ../../eurobot2024/MainRobotCode/common/acado_solver/*.cpp
                       ../../eurobot2024/MainRobotCode/common/acado_solver/*.c )
FILE( GLOB LIB_HEADERS ../../eurobot2024/MainRobotCode/common/acado_solver/*.hpp 
                       ../../eurobot2024/MainRobotCode/common/acado_solver/*.h )
set_source_files_properties(${LIB_SOURCES} PROPERTIES LANGUAGE CXX )
ADD_LIBRARY( acado_solver ${LIB_SOURCES} ${LIB_HEADERS} )
target_link_libraries( acado_solver qpoases )

message(${RPLIDAR_INCLUDE_DIRS})

# Set executable target and link libraries.
add_executable(sts_servo_test ${MAIN_SOURCES} ${COMMON_SOURCES})
target_link_libraries(sts_servo_test ${MIAM_UTILS_LIBRARIES} ${MIAM_VISION_LIBRARIES} ${RPLIDAR_LIBRARIES} ${GTK_LIBRARIES} ${HDF5LIB_LIBRARIES}
AStar acado_solver "-lhdf5_cpp")

install(TARGETS ${PROJECT_NAME} DESTINATION ".")
