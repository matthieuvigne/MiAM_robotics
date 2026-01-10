#Header directory
include_directories(
  ../common/include
  include
  ../main_robot/include
  ../embedded/include
  ../../../vision/include)

# Add all source files.
file(GLOB_RECURSE SOURCES ../common/src/*.cpp
                  ../main_robot/src/*.cpp
                  src/*.cpp)
set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CXX )

#compile flag
set(CMAKE_CXX_FLAGS "-Wall -std=c++17 -fdiagnostics-color -DSIMULATION -pthread")

# Use the package PkgConfig to detect GTK+ headers/library files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtkmm-3.0>=3.18.0)

# Include
include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})
add_definitions(${GTK3_CFLAGS_OTHER})

# Copy config.
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/config" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

# Build executables
include_directories(${CMAKE_CURRENT_BINARY_DIR}/../acado/generated "../generate_acado_solver/include")
add_executable(strategyViewer ${SOURCES})
target_link_libraries(strategyViewer miam_utils ${GTK3_LIBRARIES} acado_solver)
