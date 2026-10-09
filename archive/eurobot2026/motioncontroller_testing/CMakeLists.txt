#Header directory
include_directories(
  ../common/include
  include
  ../main_robot/include
  ../../../vision/include)

  # Add all source files.
file(GLOB_RECURSE SOURCES ../common/src/*.cpp
                  src/*.cpp)
set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CXX )


# Use the package PkgConfig to detect GTK+ headers/library files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtkmm-3.0>=3.18.0)

# Include
include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})
add_definitions(${GTK3_CFLAGS_OTHER})


# Build solver
include_directories(${CMAKE_CURRENT_BINARY_DIR}/../acado/generated)

# Copy config.
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/config" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

# Build executables
add_executable(motionControllerTesting ${SOURCES})
target_compile_options(motionControllerTesting PRIVATE -DMOTIONCONTROLLER_UNITTEST)
target_link_libraries(motionControllerTesting miam_utils acado_solver ${GTK3_LIBRARIES})
