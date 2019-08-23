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

add_library(dl_deepLearningComponent_executorlib SHARED dl_deepLearningComponent_executor.cpp dl_deepLearningComponent_executor.h)
target_link_libraries(dl_deepLearningComponent_executorlib ${LIBS})

set_source_files_properties( dl_deepLearningComponent_executor.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties( dl_deepLearningComponent_executor.i PROPERTIES SWIG_FLAGS "-ignoremissing;-w509" )

swig_add_module(dl_deepLearningComponent_executor python dl_deepLearningComponent_executor.i)
swig_link_libraries(dl_deepLearningComponent_executor dl_deepLearningComponent_executorlib ${LIBS} ${PYTHON_LIBRARIES})
