cmake_minimum_required(VERSION 3.5)

# Configure options
option(CROSS_COMPILE "True to cross compile to arm, false to compile on current platform" ON)
option(ENABLE_TELEPLOT "Enable teleplot" OFF)
option(ENABLE_TRACY "Enable tracy profiler" OFF)

if(CROSS_COMPILE)
    set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++" CACHE INTERNAL CMAKE_CXX_COMPILER)
endif()

project(miam_robotics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-Wall -fdiagnostics-color -lpthread -lm -fPIC")

if (ENABLE_TRACY)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTRACY_ENABLE")
endif()

if (ENABLE_TELEPLOT)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_TELEPLOT")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Set build-type specific flags
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -DDEBUG")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif ()

# Print options
message("Compile options:")
message("\t- Cross-compiling: " ${CROSS_COMPILE})
message("\t- Build mode: " ${CMAKE_BUILD_TYPE})
message("\t- Enable teleplot: " ${ENABLE_TELEPLOT})
message("\t- Enable tracy: " ${ENABLE_TRACY})

# Add sources
add_subdirectory(rplidar_sdk EXCLUDE_FROM_ALL)
add_subdirectory(miam_utils)
add_subdirectory(hardware_tests)
add_subdirectory(main_robot_code)