add_executable(test_as5045 TestAS5045.cpp)
target_link_libraries(test_as5045 PRIVATE miam_utils)

add_executable(test_ina226 TestINA226.cpp)
target_link_libraries(test_ina226 PRIVATE miam_utils)

add_executable(test_mcp23008 TestMCP23008.cpp)
target_link_libraries(test_mcp23008 PRIVATE miam_utils)

add_executable(test_as5048B TestAS5048B.cpp)
target_link_libraries(test_as5048B PRIVATE miam_utils)

add_executable(test_logger TestLogger.cpp)
target_link_libraries(test_logger PRIVATE miam_utils)

add_executable(test_pca9635 TestPCA9635.cpp)
target_link_libraries(test_pca9635 PRIVATE miam_utils)

add_executable(test_mainPCB TestMainPCB.cpp)
target_link_libraries(test_mainPCB PRIVATE miam_utils)

add_executable(test_BMI088 TestBMI088.cpp)
target_link_libraries(test_BMI088 PRIVATE miam_utils)

add_executable(test_SparkfunOdometry TestSparkfunOdometry.cpp)
target_link_libraries(test_SparkfunOdometry PRIVATE miam_utils)

add_executable(test_ServoPerformance TestServoPerformance.cpp)
target_link_libraries(test_ServoPerformance PRIVATE miam_utils)

add_executable(test_ServoManager TestServoManager.cpp)
target_link_libraries(test_ServoManager PRIVATE miam_utils)

# # Vision test requires opencv
# set(ENV{PKG_CONFIG_PATH} "/usr/lib/aarch64-linux-gnu/pkgconfig")
# pkg_search_module(OpenCV REQUIRED opencv4 >= 4.5)
# include_directories(${OpenCV_INCLUDE_DIRS})
# link_directories(${OpenCV_LIBRARY_DIRS})

# include_directories(../../vision/include)
# add_executable(test_camera TestSolarPanelCamera.cpp ../../vision/src/common/solar_panel_camera.cpp)
# target_link_libraries(test_camera ${GTK_LIBRARIES} PRIVATE miam_utils ${OpenCV_LIBRARIES})
