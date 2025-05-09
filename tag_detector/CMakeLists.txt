# CMake params
cmake_minimum_required(VERSION 3.5)
project(tag_detector)
set(PROJECT_DESCRIPTION "Detect aruco tags in images")

# Building options
set(CMAKE_CXX_COMPILER "g++")
add_compile_options(-std=c++17)

# Set build-type specific flags
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: debug.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -DDEBUG")
else (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Build mode: release build - optimization enabled.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

# Link to external libraries
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(OpenCV REQUIRED opencv4 >= 4.5)
pkg_search_module(Camera REQUIRED libcamera)

include_directories("${OpenCV_INCLUDE_DIRS}" "${Camera_INCLUDE_DIRS}" ".")
link_directories("${OpenCV_DIRS} ${Camera_DIRS}")

add_executable(tag_detector src/run_tag_detection.cpp src/LibCamera.cpp)
target_link_libraries(tag_detector camera ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${Camera_LIBRARIES})


