cmake_minimum_required(VERSION 3.5.1)

set (CMAKE_CXX_STANDARD 14)

find_package(SWIG REQUIRED)
find_package(Armadillo REQUIRED)
FIND_PACKAGE(PythonLibs 2.7 REQUIRED)

<#noparse>
include(${SWIG_USE_FILE})

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} "armanpy" "..")
set(LIBS ${LIBS} ${ARMADILLO_LIBRARIES})
set( CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}" )

include_directories(${INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} .)
</#noparse>

add_library(${wrapperClassName}lib SHARED ${wrapperClassName}.cpp ${wrapperClassName}.h)
target_link_libraries(${wrapperClassName}lib <#noparse>${LIBS}</#noparse>)

set_source_files_properties( ${wrapperClassName}.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties( ${wrapperClassName}.i PROPERTIES SWIG_FLAGS "-ignoremissing;-w509" )

swig_add_module(${wrapperClassName} python ${wrapperClassName}.i)
swig_link_libraries(${wrapperClassName} ${wrapperClassName}lib <#noparse>${LIBS} ${PYTHON_LIBRARIES}</#noparse>)
