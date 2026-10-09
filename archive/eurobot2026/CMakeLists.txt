add_subdirectory("generate_acado_solver" "acado/")


# Build 'local' acado solver
include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated
					../qpoases
					../qpoases/INCLUDE
					../qpoases/SRC)
FILE(GLOB QPOASES_SOURCES ../qpoases/SRC/*.cpp ../qpoases/SRC/*.hpp )
set_source_files_properties(${ACADO_GEN_FILES} PROPERTIES LANGUAGE CXX )

add_library(acado_solver ${ACADO_GEN_FILES} ${QPOASES_SOURCES})
add_dependencies(acado_solver acado_codegen)
target_include_directories(acado_solver PUBLIC ${ACADO_GEN_INCLUDE_DIR})
target_include_directories(acado_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/generate_acado_solver/include)


if (CROSS_COMPILE)
    add_subdirectory(embedded)
    add_subdirectory(servo_test)
else()
    add_subdirectory(simulation)
    add_subdirectory(motioncontroller_testing)
endif()
