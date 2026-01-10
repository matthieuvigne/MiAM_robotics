include_directories(include)
file(GLOB_RECURSE SOURCES src/*.cpp)

# Use the package PkgConfig to detect GTK+ headers/library files - cross-compiled
find_package(PkgConfig REQUIRED)
unset(GTK_FOUND CACHE)
if(CROSS_COMPILE)
    set(ENV{PKG_CONFIG_PATH} "/usr/lib/aarch64-linux-gnu/pkgconfig")
else()
    pkg_search_module(MIAM_UTILS REQUIRED miam_utils >= 2.5.4)
endif()
pkg_search_module(GTK REQUIRED gtkmm-3.0)


include_directories(${GTK_INCLUDE_DIRS})
link_directories(${GTK_LIBRARY_DIRS})

# Build executables
add_executable(sts_servo_gui ${SOURCES})
target_link_libraries(sts_servo_gui miam_utils ${GTK_LIBRARIES})
