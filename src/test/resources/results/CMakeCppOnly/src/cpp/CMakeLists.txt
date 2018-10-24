cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)

project(tests_a_addComp LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})

# additional commands

# create static library
include_directories(${INCLUDE_DIRS})
add_library(tests_a_addComp tests_a_addComp.h)
target_include_directories(tests_a_addComp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(tests_a_addComp PUBLIC ${LIBS})
set_target_properties(tests_a_addComp PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS tests_a_addComp FILE tests_a_addComp.cmake)

# additional commands end
