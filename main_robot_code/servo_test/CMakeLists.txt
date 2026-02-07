# Find GTK+ headers/library files - cross-compiled
find_package(PkgConfig REQUIRED)
set(ENV{PKG_CONFIG_PATH} "/usr/lib/aarch64-linux-gnu/pkgconfig")
pkg_search_module(GTK REQUIRED gtkmm-3.0)

include_directories(
  ${GTK_INCLUDE_DIRS}
  )
link_directories(
  ${GTK_LIBRARY_DIRS}
  )

# Include source files from src directory.
file(GLOB_RECURSE COMMON_SOURCES "../common/src/*.cpp"
                                 "../embedded/src/Robot.cpp")

file(GLOB_RECURSE MAIN_SOURCES "../main_robot/src/*.cpp" "src/sts_servo_test.cpp")

set_source_files_properties(${${PROJECT_NAME}_SOURCES} PROPERTIES LANGUAGE CXX )

# Add directory containing the headers.
include_directories(../common/include)
include_directories(../../../tracy/public)
include_directories(../main_robot/include)
include_directories(../embedded/include)

# Set executable target and link libraries.
add_executable(sts_servo_test ${MAIN_SOURCES} ${COMMON_SOURCES})

target_link_libraries(sts_servo_test
  miam_utils
  acado_solver
  ${GTK_LIBRARIES})