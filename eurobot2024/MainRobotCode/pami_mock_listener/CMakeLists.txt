#minimum CMake Version
cmake_minimum_required(VERSION 3.2)
#project name
project(mockListener)
#g++ color output
SET(CMAKE_COLOR_MAKEFILE ON)
#Header directory
include_directories(
  # ../common/include
  include
  # ../main_robot/include
  # # ../embedded/include
  # # ../../../vision/include
  # ../../../qpoases
  # ../../../qpoases/INCLUDE
  # ../../../qpoases/SRC
  # ../common/acado_solver
)

# Add all source files.
file(GLOB_RECURSE SOURCES 
                  # ../common/src/*.cpp
                  # ../main_robot/src/*.cpp
                  src/*.cpp
                  # ../../../vision/src/network/camera_client.cpp
                  # ../../../vision/src/network/client*.cpp
                  # ../../../vision/src/network/socket.cpp
                  # ../../../vision/src/common/maths.cpp
                  # ../../../vision/src/common/logger.cpp
                  # ../../../vision/src/common/marker.cpp
                  # ../../../vision/src/network/server_response.cpp
                  # ../../../vision/src/network/message.cpp
                  )
set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CXX )

#compile flag
set(CMAKE_CXX_FLAGS "-Wall -std=c++17 -fdiagnostics-color -DSIMULATION -pthread")

# Use the package PkgConfig to detect GTK+ headers/library files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtkmm-3.0>=3.18.0)
pkg_search_module(HDF5LIB REQUIRED hdf5)
find_package(Eigen3 REQUIRED)

unset(MIAM_UTILS_FOUND CACHE)
pkg_search_module(RPLIDAR REQUIRED rplidar)
message(STATUS "Found rplidar v." ${RPLIDAR_VERSION})
pkg_search_module(MIAM_UTILS REQUIRED miam_utils>=2.7.0)
message(STATUS "Found miam_utils v." ${MIAM_UTILS_VERSION})
include_directories(${Eigen_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS} ${MIAM_UTILS_INCLUDE_DIRS} ${HDF5LIB_INCLUDE_DIRS})
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS} ${HDF5LIB_LIBRARY_DIRS})

# Include
include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})
add_definitions(${GTK3_CFLAGS_OTHER})
# include_directories(../../../a-star/source)

# FILE( GLOB LIB_SOURCES ../../../a-star/source/*.cpp )
# FILE( GLOB LIB_HEADERS ../../../a-star/source/*.hpp )
# include_directories(../../../a-star/source)
# ADD_LIBRARY( AStar ${LIB_SOURCES} ${LIB_HEADERS} )

# # qpoases
# FILE( GLOB LIB_SOURCES ../../../qpoases/SRC/*.cpp )
# FILE( GLOB LIB_HEADERS ../../../qpoases/INCLUDE/*.hpp )
# ADD_LIBRARY( qpoases ${LIB_SOURCES} ${LIB_HEADERS} )

# # acado_solver
# FILE( GLOB LIB_SOURCES ../common/acado_solver/*.cpp ../common/acado_solver/*.c )
# FILE( GLOB LIB_HEADERS ../common/acado_solver/*.hpp ../common/acado_solver/*.h )
# ADD_LIBRARY( acado_solver ${LIB_SOURCES} ${LIB_HEADERS} )
# target_link_libraries( acado_solver qpoases)

# # Copy config.
# file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/config" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

# Build executables
add_executable(mockListener ${SOURCES})
target_link_libraries(
  mockListener ${GTK3_LIBRARIES} ${RPLIDAR_LIBRARIES} ${MIAM_UTILS_LIBRARIES} ${HDF5LIB_LIBRARIES} 
  # AStar acado_solver 
  "-lhdf5_cpp")
