cmake_minimum_required(VERSION 3.5)
project(RNNsearch CXX)

set(CMAKE_CXX_STANDARD 11)

add_executable(RNNsearch RNNsearch.cpp)

set_target_properties(RNNsearch PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
target_link_libraries(RNNsearch ${MXNET_PATH})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(RNNsearch rnnencdec_main ${LIBS} -lblas -lopenblas)
target_include_directories(RNNsearch PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS RNNsearch FILE RNNsearch.cmake)

