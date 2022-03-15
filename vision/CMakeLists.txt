# CMake params
cmake_minimum_required(VERSION 2.8.3)
project(vision)

# Building options
#~ set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-std=c++11)

# Library
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(Threads REQUIRED)
add_library(${PROJECT_NAME}
  src/camera.cpp
  src/camera_pose_filter.cpp
  src/client.cpp
  src/client_socket.cpp
  src/distortion_model.cpp
  src/distortion_null.cpp
  src/distortion_radtan.cpp
  src/module.cpp
  src/server.cpp
  src/server_socket.cpp
  src/socket.cpp)
include_directories("include")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} yaml-cpp ${CMAKE_THREAD_LIBS_INIT})
link_directories(${Eigen_INCLUDE_DIRS})

# Executable files

add_executable(test_geometry scripts/test_geometry.cpp)
target_link_libraries(test_geometry ${Eigen_LIBRARIES})

#~ add_executable(test_fov scripts/test_fov.cpp)
#~ target_link_libraries(test_fov ${Eigen_LIBRARIES})

#~ add_executable(client scripts/client.cpp)
#~ add_executable(server scripts/server.cpp)

#~ add_executable(test_board scripts/test_board.cpp)
#~ target_link_libraries(test_board ${OpenCV_LIBS} ${Eigen_LIBRARIES} ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

#~ add_executable(test_aruco scripts/test_aruco.cpp)
#~ target_link_libraries(test_aruco ${OpenCV_LIBS} ${Eigen_LIBRARIES})

#~ add_executable(create_aruco_video scripts/create_aruco_video.cpp)
#~ target_link_libraries(create_aruco_video ${OpenCV_LIBS} ${Eigen_LIBRARIES})
