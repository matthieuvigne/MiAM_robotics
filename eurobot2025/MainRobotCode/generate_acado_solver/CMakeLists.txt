# Generate acado code for MPC solver

# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 3.10 )

# Project name and programming languages used
PROJECT("acado_codegen" CXX C)

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
# Ignore ignored-attributes (ACADO warnings) & deprecated-declarations (Eigen warnings)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -Wno-ignored-attributes -Wno-deprecated-declarations")
FIND_PACKAGE(ACADO REQUIRED )
include_directories(${ACADO_INCLUDE_DIRS} "include")

ADD_EXECUTABLE("acado_codegen" "src/mpc_code_generation.cpp")
TARGET_LINK_LIBRARIES("acado_codegen" ${ACADO_SHARED_LIBRARIES})

# Create fake files if needed
set(ACADO_GEN_FILES generated/acado_auxiliary_functions.c
			        generated/acado_common.h
			        generated/acado_qpoases_interface.cpp
			        generated/acado_solver.c
			        generated/acado_auxiliary_functions.h
			        generated/acado_integrator.c
			        generated/acado_qpoases_interface.hpp)

file(MAKE_DIRECTORY generated)
execute_process(
  COMMAND ${CMAKE_COMMAND} -E make_directory generated
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )

execute_process(
  COMMAND ${CMAKE_COMMAND} -E touch ${ACADO_GEN_FILES}
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )

# Regenerate files if needed
add_custom_command(
  TARGET "acado_codegen" POST_BUILD
  COMMAND "./acado_codegen"
  DEPENDS "acado_codegen"
)

if(NOT DEFINED ${NO_ACADO})
	# Build acado solver
	include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated
						../../../qpoases
						../../../qpoases/INCLUDE
						../../../qpoases/SRC)
	FILE(GLOB QPOASES_SOURCES ../../../qpoases/SRC/*.cpp ../../../qpoases/SRC/*.hpp )
	set_source_files_properties(${ACADO_GEN_FILES} PROPERTIES LANGUAGE CXX )

	ADD_LIBRARY(acado_solver ${ACADO_GEN_FILES} ${QPOASES_SOURCES})
	add_dependencies(acado_solver acado_codegen)
endif()