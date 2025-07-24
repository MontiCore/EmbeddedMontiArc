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

add_library(calculator_calculator_executorlib SHARED calculator_calculator_executor.cpp calculator_calculator_executor.h)
target_link_libraries(calculator_calculator_executorlib ${LIBS})

set_source_files_properties( calculator_calculator_executor.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties( calculator_calculator_executor.i PROPERTIES SWIG_FLAGS "-ignoremissing;-w509" )

swig_add_module(calculator_calculator_executor python calculator_calculator_executor.i)
swig_link_libraries(calculator_calculator_executor calculator_calculator_executorlib ${LIBS} ${PYTHON_LIBRARIES})
