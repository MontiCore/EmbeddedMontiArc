cmake_minimum_required(VERSION 3.5)
project(MNISTGenerator CXX)
set(CMAKE_CXX_STANDARD 11)

option(MXNET_PATH "Path to libmxnet.so")


add_executable(MNISTGenerator MNISTGenerator.cpp)

set_target_properties(MNISTGenerator PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
target_link_libraries(MNISTGenerator ${MXNET_PATH})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(MNISTGenerator  mnistGenerator_connector ${LIBS} -lblas -lopenblas)
target_include_directories(MNISTGenerator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS MNISTGenerator FILE MNISTGenerator.cmake)
