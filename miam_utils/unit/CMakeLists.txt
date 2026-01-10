include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  EXCLUDE_FROM_ALL
)
FetchContent_GetProperties(googletest)
if(NOT googletest_POPULATED)
  FetchContent_Populate(googletest)
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

include_directories(${HDF5LIB_INCLUDE_DIRS})
link_directories(${HDF5LIB_LIBRARY_DIRS})

# Now simply link against gtest or gtest_main as needed. Eg
file(GLOB_RECURSE SOURCES "*.cc")
add_executable(unit ${SOURCES})

target_link_libraries(unit gtest_main miam_utils ${RPLIDARLIB_LIBRARIES} ${HDF5LIB_LIBRARIES} hdf5_cpp)
add_test(NAME unit COMMAND unit)
