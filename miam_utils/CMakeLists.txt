
# Set compiler and library name.
if(CROSS_COMPILE)
    set(PUBLIC_LIB_NAME miam_utils_arm)
else()
    set(PUBLIC_LIB_NAME miam_utils)
    add_subdirectory(log_converter)
    add_subdirectory(unit)
endif()

# Specify library content :
# Recursively include all .c files
file(GLOB_RECURSE LIB_SOURCES "src/*.c"
                              "src/*.cpp"
                              "../tracy/public/TracyClient.cpp")
set_source_files_properties(${LIB_SOURCES} PROPERTIES LANGUAGE CXX )

# Setup static library
add_library(miam_utils STATIC ${LIB_SOURCES})
target_include_directories(miam_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(miam_utils PUBLIC ../tracy/public/)
target_include_directories(miam_utils PUBLIC ../teleplot/clients/cpp)
target_link_libraries(miam_utils PUBLIC rplidar)

# Create package config file from template.

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/miam_utilsTemplate.pc" "${CMAKE_CURRENT_BINARY_DIR}/${PUBLIC_LIB_NAME}.pc")

# Set install rules: copy library and headers.
install(TARGETS miam_utils DESTINATION "lib/${PUBLIC_LIB_NAME}.a")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/" DESTINATION "include")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../teleplot/clients/cpp/" DESTINATION "include")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../tracy/public/" DESTINATION "include/")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PUBLIC_LIB_NAME}.pc" DESTINATION "lib/pkgconfig/")
