cmake_minimum_required(VERSION 3.5.1)

set (CMAKE_CXX_STANDARD 14)

find_package(SWIG REQUIRED)
find_package(Armadillo REQUIRED)
FIND_PACKAGE(PythonLibs 2.7 REQUIRED)

include(${SWIG_USE_FILE})

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} "armanpy" "..")
set(LIBS ${LIBS} ${ARMADILLO_LIBRARIES})
set( CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}" )

include_directories(${INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} .)

add_library(test_noInputComponent_executorlib SHARED test_noInputComponent_executor.cpp test_noInputComponent_executor.h)
target_link_libraries(test_noInputComponent_executorlib ${LIBS})

set_source_files_properties( test_noInputComponent_executor.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties( test_noInputComponent_executor.i PROPERTIES SWIG_FLAGS "-ignoremissing;-w509" )

swig_add_module(test_noInputComponent_executor python test_noInputComponent_executor.i)
swig_link_libraries(test_noInputComponent_executor test_noInputComponent_executorlib ${LIBS} ${PYTHON_LIBRARIES})
