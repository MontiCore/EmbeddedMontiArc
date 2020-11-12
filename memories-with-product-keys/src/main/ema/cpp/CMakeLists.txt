cmake_minimum_required(VERSION 3.5)
project(MemoriesWithProductKeys CXX)
set(CMAKE_CXX_STANDARD 11)

option(MXNET_PATH "Path to libmxnet.so")


add_executable(MemoriesWithProductKeys MemoriesWithProductKeys.cpp)

set_target_properties(MemoriesWithProductKeys PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(MemoriesWithProductKeys ${MXNET_PATH})
target_link_libraries(MemoriesWithProductKeys memorieswithproductkeys_connector ${LIBS} -lblas -lopenblas)
target_include_directories(MemoriesWithProductKeys PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS MemoriesWithProductKeys FILE MemoriesWithProductKeys.cmake)
