cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(de_rwth_monticar_optimization_hS71 LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})

find_package(CPPAD REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CPPAD_INCLUDE_DIRS})


find_package(Ipopt REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Ipopt_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Ipopt_LIBRARIES})


# additional library linkage

# additional commands

# create static library
include_directories(${INCLUDE_DIRS})
add_library(de_rwth_monticar_optimization_hS71 de_rwth_monticar_optimization_hS71.cpp)
target_include_directories(de_rwth_monticar_optimization_hS71 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(de_rwth_monticar_optimization_hS71 PUBLIC ${LIBS})
set_target_properties(de_rwth_monticar_optimization_hS71 PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS de_rwth_monticar_optimization_hS71 FILE de_rwth_monticar_optimization_hS71.cmake)

# additional commands end
