cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(endtoend_nvidia LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})

# additional commands
find_package(TensorflowCC REQUIRED)
set(LIBS ${LIBS} TensorflowCC::Shared)

# create static library
include_directories(${INCLUDE_DIRS})
add_library(endtoend_nvidia endtoend_nvidia.cpp)
target_include_directories(endtoend_nvidia PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(endtoend_nvidia PUBLIC ${LIBS})
set_target_properties(endtoend_nvidia PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS endtoend_nvidia FILE endtoend_nvidia.cmake)

# additional commands end
