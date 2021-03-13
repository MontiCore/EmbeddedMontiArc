cmake_minimum_required(VERSION 3.5)
project(emacodebert CXX)
set(CMAKE_CXX_STANDARD 11)

option(MXNET_PATH "Path to libmxnet.so")


add_executable(emacodebert emacodebert.cpp)

set_target_properties(emacodebert PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(emacodebert ${MXNET_PATH})
target_link_libraries(emacodebert emacodebert_connector ${LIBS} -lblas -lopenblas)
target_include_directories(emacodebert PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS emacodebert FILE emacodebert.cmake)
