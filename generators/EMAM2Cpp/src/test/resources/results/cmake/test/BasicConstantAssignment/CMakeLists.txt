cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(test_basicConstantAssignment LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})

# additional library linkage

# additional commands

# create static library
include_directories(${INCLUDE_DIRS})
add_library(test_basicConstantAssignment test_basicConstantAssignment.cpp)
target_include_directories(test_basicConstantAssignment PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(test_basicConstantAssignment PUBLIC ${LIBS})
set_target_properties(test_basicConstantAssignment PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS test_basicConstantAssignment FILE test_basicConstantAssignment.cmake)

# additional commands end
