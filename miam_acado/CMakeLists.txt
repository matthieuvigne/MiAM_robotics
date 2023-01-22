#
# Project settings
#

# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 3.10 )

# Project name and programming languages used
PROJECT( CODE_GENERATION_MPC CXX C)


# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
message(${CMAKE_MODULE_PATH})
# Ignore ignored-attributes (ACADO warnings) & deprecated-declarations (Eigen warnings)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-attributes -Wno-deprecated-declarations")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-psabi -fdiagnostics-color")

# #
# # Include directories
# #
# INCLUDE_DIRECTORIES( ../RobotCode ${ACADO_INCLUDE_DIRS} )

#
# Set library up
#

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)


#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )
pkg_search_module(ACADO REQUIRED rplidar)
message(STATUS "Found acado v." ${ACADO_VERSION})


pkg_search_module(RPLIDAR REQUIRED rplidar)
message(STATUS "Found rplidar v." ${RPLIDAR_VERSION})
pkg_search_module(MIAM_UTILS REQUIRED miam_utils >= 2.7.0)
message(STATUS "Found miam_utils v." ${MIAM_UTILS_VERSION})

message(${MIAM_UTILS_INCLUDE_DIRS})
message(${MIAM_UTILS_LIBRARY_DIRS})

include_directories(
	${Eigen_INCLUDE_DIRS} 
	${RPLIDAR_INCLUDE_DIRS} 
	${MIAM_UTILS_INCLUDE_DIRS} 
	${ACADO_INCLUDE_DIRS}
	src/qpoases
	src/qpoases/INCLUDE
	src/qpoases/SRC
	src/generated_code
)
link_directories(${RPLIDAR_LIBRARY_DIRS} ${MIAM_UTILS_LIBRARY_DIRS})

# Copy config.
# file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/config" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")


# FILE( GLOB LIB_SOURCES ../RobotCode/MiAMEurobot/src/trajectory/DrivetrainKinematics.cpp )
# FILE( GLOB LIB_HEADERS ../RobotCode/MiAMEurobot/include/trajectory/DrivetrainKinematics.h )
# ADD_LIBRARY( DrivetrainKinematics ${LIB_SOURCES} ${LIB_HEADERS} )


# QPoases
FILE( GLOB LIB_SOURCES src/qpoases/SRC/*.cpp )
FILE( GLOB LIB_HEADERS src/qpoases/INCLUDE/*.hpp )
ADD_LIBRARY( QPoases ${LIB_SOURCES} ${LIB_HEADERS} )

# CustomSolver
FILE( GLOB LIB_SOURCES src/generated_code/*.cpp src/generated_code/*.c )
FILE( GLOB LIB_HEADERS src/generated_code/*.hpp src/generated_code/*.h )
ADD_LIBRARY( CustomSolver ${LIB_SOURCES} ${LIB_HEADERS} )

#
# Build executables
#
FILE( GLOB TEST_SOURCES src/*.cpp )

FOREACH( TEST_SOURCE_FILE ${TEST_SOURCES} )

	GET_FILENAME_COMPONENT( TEST_NAME ${TEST_SOURCE_FILE} NAME_WE )
	ADD_EXECUTABLE( ${TEST_NAME} ${TEST_SOURCE_FILE} )
	TARGET_LINK_LIBRARIES( 
		${TEST_NAME} 
		${ACADO_SHARED_LIBRARIES} 
		${RPLIDAR_LIBRARIES} 
		${MIAM_UTILS_LIBRARIES} 
		CustomSolver 
		QPoases 
	)

ENDFOREACH( TEST_SOURCE_FILE ${TEST_SOURCES} )

