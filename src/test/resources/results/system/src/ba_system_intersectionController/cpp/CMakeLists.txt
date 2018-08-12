cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)

project(ba_system_intersectionController LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})

# additional commands

# create static library
include_directories(${INCLUDE_DIRS})
add_library(ba_system_intersectionController ba_system_intersectionController.h)
target_include_directories(ba_system_intersectionController PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ba_system_intersectionController PUBLIC ${LIBS})
set_target_properties(ba_system_intersectionController PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS ba_system_intersectionController FILE ba_system_intersectionController.cmake)

# additional commands end
