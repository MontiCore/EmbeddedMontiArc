cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(de_rwth_monticar_optimization_colRowMinTest LANGUAGES CXX)

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
add_library(de_rwth_monticar_optimization_colRowMinTest de_rwth_monticar_optimization_colRowMinTest.cpp)
target_include_directories(de_rwth_monticar_optimization_colRowMinTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(de_rwth_monticar_optimization_colRowMinTest PUBLIC ${LIBS})
set_target_properties(de_rwth_monticar_optimization_colRowMinTest PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS de_rwth_monticar_optimization_colRowMinTest FILE de_rwth_monticar_optimization_colRowMinTest.cmake)

# additional commands end
