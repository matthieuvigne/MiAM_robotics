# Generate 'acado_codegen' binary

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

# Ignore ignored-attributes (ACADO warnings) & deprecated-declarations (Eigen warnings)
FIND_PACKAGE(ACADO REQUIRED )
include_directories(${ACADO_INCLUDE_DIRS} "include")

set(OLD_COMPILER ${CMAKE_CXX_COMPILER})
set(CMAKE_CXX_COMPILER "g++")

add_executable(acado_codegen "src/mpc_code_generation.cpp")
target_link_libraries(acado_codegen ${ACADO_SHARED_LIBRARIES})

# Create fake files if needed
set(ACADO_GEN_FILES ${CMAKE_CURRENT_BINARY_DIR}/generated/acado_auxiliary_functions.c
			        ${CMAKE_CURRENT_BINARY_DIR}/generated/acado_common.h
			        ${CMAKE_CURRENT_BINARY_DIR}/generated/acado_qpoases_interface.cpp
			        ${CMAKE_CURRENT_BINARY_DIR}/generated/acado_solver.c
			        ${CMAKE_CURRENT_BINARY_DIR}/generated/acado_auxiliary_functions.h
			        ${CMAKE_CURRENT_BINARY_DIR}/generated/acado_integrator.c
			        ${CMAKE_CURRENT_BINARY_DIR}/generated/acado_qpoases_interface.hpp
	CACHE INTERNAL ACADO_GEN_FILES)
set(ACADO_GEN_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/
	CACHE INTERNAL ACADO_GEN_INCLUDE_DIR)

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
  TARGET acado_codegen POST_BUILD
  COMMAND "./acado_codegen"
  DEPENDS acado_codegen
)

# set(CMAKE_CXX_COMPILER ${OLD_COMPILER})

# # Build acado solver
# include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated
# 					../../qpoases
# 					../../qpoases/INCLUDE
# 					../../qpoases/SRC)
# FILE(GLOB QPOASES_SOURCES ../../qpoases/SRC/*.cpp ../../qpoases/SRC/*.hpp )
# set_source_files_properties(${ACADO_GEN_FILES} PROPERTIES LANGUAGE CXX )

# add_library(acado_solver ${ACADO_GEN_FILES} ${QPOASES_SOURCES})
# add_dependencies(acado_solver acado_codegen)
# target_include_directories(acado_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
