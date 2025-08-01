cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(alexnet LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})


# additional library linkage
set(LIBS ${LIBS} TensorflowCC::Shared)

# additional commands
find_package(TensorflowCC REQUIRED)

# create static library
include_directories(${INCLUDE_DIRS})
add_library(alexnet alexnet.cpp)
target_include_directories(alexnet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(alexnet PUBLIC ${LIBS})
set_target_properties(alexnet PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS alexnet FILE alexnet.cmake)

# additional commands end
