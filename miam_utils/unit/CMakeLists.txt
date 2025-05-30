# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.main
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

include_directories(${RPLIDARLIB_INCLUDE_DIRS} ${HDF5LIB_INCLUDE_DIRS})
link_directories(${RPLIDARLIB_LIBRARY_DIRS} ${HDF5LIB_LIBRARY_DIRS})

# Now simply link against gtest or gtest_main as needed. Eg
file(GLOB_RECURSE SOURCES "*.cc")
add_executable(unit ${SOURCES})
include_directories(BEFORE "../include")

target_link_libraries(unit gtest_main miam_utils ${RPLIDARLIB_LIBRARIES} ${HDF5LIB_LIBRARIES} hdf5_cpp)
add_test(NAME unit COMMAND unit)
