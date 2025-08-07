cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(alexnet LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# add dependencies
find_package(Armadillo REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Armadillo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Armadillo_LIBRARIES})

find_package(Caffe2 REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Caffe2_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Caffe2_LIBRARIES})


# additional library linkage

# additional commands
set(LIBS ${LIBS} -lprotobuf -lglog -lgflags)
find_package(CUDA)

if(CUDA_FOUND)
  set(LIBS ${LIBS} caffe2 caffe2_gpu)
  set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${CUDA_LIBRARIES} ${CUDA_curand_LIBRARY})
else()
  set(LIBS ${LIBS} caffe2)
endif()

# create static library
include_directories(${INCLUDE_DIRS})
add_library(alexnet alexnet.cpp)
target_include_directories(alexnet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_DIRS})
target_link_libraries(alexnet PUBLIC ${LIBS})
set_target_properties(alexnet PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS alexnet FILE alexnet.cmake)

# additional commands end
