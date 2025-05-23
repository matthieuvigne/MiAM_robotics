cmake_minimum_required(VERSION 3.5)

#Configure project
project(miam_utils VERSION 2.9.0)
set(PROJECT_DESCRIPTION "C++ utility library to use a Beaglebone Black / Raspberry Pi for Eurobot - low-level drivers and some higher level utilities")

# Option to choose between cross-compiling or compiling for local computer.
# If compiling for the current computer, _intel is appended to the library name.

option(CROSS_COMPILE "True to cross compile to arm, false to compile on current platform" ON)
option(UNIT_TESTS "Enable unit tests (ignored when cross-compiling)" OFF)
option(ENABLE_TELEPLOT "Enable unit tests (ignored when cross-compiling)" OFF)
option(ENABLE_TRACY "Enable tracy profiler" OFF)

# Set compiler and library name.

if(CROSS_COMPILE)
    set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
    set(LIBRARY_NAME ${PROJECT_NAME}_arm)
else()
    add_subdirectory(log_converter)
    # Add unit test only if not cross-compiling
    if(UNIT_TESTS)
        add_subdirectory(unit)
    endif()
    set(CMAKE_CXX_COMPILER "g++")
    set(LIBRARY_NAME ${PROJECT_NAME})
endif()

# Set compiler flags.
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -fdiagnostics-color -lpthread -lm")

if (ENABLE_TRACY)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTRACY_ENABLE")
endif()

if (ENABLE_TELEPLOT)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_TELEPLOT")
endif()


# Set build-type specific flags
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: debug.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -DDEBUG")
else (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: release build - optimization enabled.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

# Specify library content :
# Recursively include all .c files
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.c"
                                          "src/*.cpp"
                                          "../tracy/public/TracyClient.cpp")
set_source_files_properties(${${PROJECT_NAME}_SOURCES} PROPERTIES LANGUAGE CXX )

# Add directory containing the headers, and put all headers in a variable.
include_directories(include)

include_directories(../teleplot/clients/cpp)
include_directories(../tracy/public/)

# Setup static library
add_library(${LIBRARY_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# Use pkg-config to link to external library.
find_package(PkgConfig REQUIRED)
unset(RPLIDARLIB_FOUND CACHE)
# Link to rplidar library (take the correct version depending on wheather we are cross-compiling or not).
if(CROSS_COMPILE)
    pkg_search_module(RPLIDARLIB REQUIRED rplidar_arm)
    set(ENV{PKG_CONFIG_PATH} "/usr/lib/arm-linux-gnueabihf/pkgconfig")
else()
    pkg_search_module(RPLIDARLIB REQUIRED rplidar)
endif()
message(STATUS "Found rplidar v." ${RPLIDARLIB_VERSION})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

include_directories(${RPLIDARLIB_INCLUDE_DIRS})
link_directories(${RPLIDARLIB_LIBRARY_DIRS})

target_link_libraries(${LIBRARY_NAME} ${RPLIDARLIB_LIBRARIES})

# Create package config file from template.
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/miam_utilsTemplate.pc" "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}.pc")

# Set install rules: copy library and headers.
install(TARGETS ${LIBRARY_NAME} DESTINATION "lib")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/" DESTINATION "include")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../teleplot/clients/cpp/" DESTINATION "include")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../tracy/public/" DESTINATION "include/")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}.pc" DESTINATION "lib/pkgconfig/")
