# Specify library content :
# Recursively include all .c files
file(GLOB_RECURSE LOG_CONVERTER_SOURCES "src/*.cpp")

# Add directory containing the headers, and put all headers in a variable.
include_directories(include)

# Use pkg-config to link to external library.
find_package(PkgConfig REQUIRED)
pkg_search_module(HDF5LIB REQUIRED hdf5)

include_directories(${HDF5LIB_INCLUDE_DIRS})
link_directories(${HDF5LIB_LIBRARY_DIRS})

add_executable(miam_log_converter ${LOG_CONVERTER_SOURCES})
target_link_libraries(miam_log_converter ${HDF5LIB_LIBRARIES} "-lhdf5_cpp")

install(TARGETS miam_log_converter DESTINATION "bin")