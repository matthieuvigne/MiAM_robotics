include_directories(include)
file(GLOB SOURCES src/*.cpp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtkmm-3.0>=3.18.0)

include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})
add_definitions(${GTK3_CFLAGS_OTHER})

# Build executables
add_executable(rplidarTester ${SOURCES})
target_link_libraries(rplidarTester miam_utils ${GTK3_LIBRARIES})

add_custom_command(
        TARGET rplidarTester POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_SOURCE_DIR}/config/GRPLidar.glade
                ${CMAKE_CURRENT_BINARY_DIR}/GRPLidar.glade)