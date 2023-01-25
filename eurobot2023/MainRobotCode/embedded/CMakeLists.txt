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

# Use pkg-config to find rplidar
find_package(PkgConfig REQUIRED)
pkg_search_module(RPLIDAR REQUIRED rplidar_arm)
message(STATUS "Found rplidar_arm v." ${RPLIDAR_VERSION})
unset(MIAM_UTILS_FOUND CACHE)
pkg_search_module(MIAM_UTILS REQUIRED miam_utils_arm >= 2.7.0)
message(STATUS "Found miam_utils_arm v." ${MIAM_UTILS_VERSION})

set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib)

# Find GTK+ headers/library files - cross-compiled
set(ENV{PKG_CONFIG_PATH} "/usr/lib/arm-linux-gnueabihf/pkgconfig")
pkg_search_module(GTK REQUIRED gtkmm-3.0)
pkg_search_module(HDF5LIB REQUIRED hdf5-serial)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS} ${MIAM_UTILS_INCLUDE_DIRS} ${GTK_INCLUDE_DIRS} ${HDF5LIB_INCLUDE_DIRS})
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS} ${GTK_LIBRARY_DIRS} ${HDF5LIB_LIBRARY_DIRS})

# Include source files from src directory.
file(GLOB_RECURSE COMMON_SOURCES "../common/src/*.cpp"
                                 "src/uCListener.cpp"
                                 "src/Robot.cpp"
                                 "../../../vision/src/network/camera_client.cpp"
                                 "../../../vision/src/network/client_request.cpp"
                                 "../../../vision/src/network/client_socket.cpp"
                                 "../../../vision/src/network/socket.cpp"
                                 "../../../vision/src/common/maths.cpp"
                                 "../../../vision/src/common/logger.cpp"
                                 "../../../vision/src/common/marker.cpp"
                                 "../../../vision/src/network/server_response.cpp"
                                 "../../../vision/src/network/message.cpp")

file(GLOB_RECURSE MAIN_SOURCES "../main_robot/src/*.cpp" "src/MainRobotMain.cpp")

file(GLOB_RECURSE SECONDARY_SOURCES "../secondary_robot/src/*.cpp" "src/SecondaryRobotMain.cpp")


set_source_files_properties(${${PROJECT_NAME}_SOURCES} PROPERTIES LANGUAGE CXX )

# Add directory containing the headers.
include_directories(../common/include)
include_directories(../main_robot/include)
include_directories(../secondary_robot/include)
include_directories(./include)
include_directories(../../../vision/include)

message(${RPLIDAR_INCLUDE_DIRS})

# Set executable target and link libraries.
add_executable(mainRobotCode ${MAIN_SOURCES} ${COMMON_SOURCES})
target_link_libraries(mainRobotCode ${MIAM_UTILS_LIBRARIES} ${RPLIDAR_LIBRARIES} ${GTK_LIBRARIES} ${HDF5LIB_LIBRARIES} "-lhdf5_cpp")


add_executable(secondaryRobotCode ${SECONDARY_SOURCES} ${COMMON_SOURCES})
target_link_libraries(secondaryRobotCode ${MIAM_UTILS_LIBRARIES} ${RPLIDAR_LIBRARIES} ${GTK_LIBRARIES} ${HDF5LIB_LIBRARIES} "-lhdf5_cpp")

install(TARGETS ${PROJECT_NAME} DESTINATION ".")