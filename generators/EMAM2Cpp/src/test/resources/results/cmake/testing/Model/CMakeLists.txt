cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(testing_model LANGUAGES CXX)

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
add_library(testing_model testing_model.cpp)
target_include_directories(testing_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(testing_model PUBLIC ${LIBS})
set_target_properties(testing_model PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS testing_model FILE testing_model.cmake)

# additional commands end
