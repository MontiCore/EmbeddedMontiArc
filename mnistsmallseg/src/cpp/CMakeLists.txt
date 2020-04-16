cmake_minimum_required(VERSION 3.5)
project(DigitSegment CXX)
set(CMAKE_CXX_STANDARD 11)

option(MXNET_PATH "Path to libmxnet.so")


add_executable(DigitSegment DigitSegment.cpp)

set_target_properties(DigitSegment PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
target_link_libraries(DigitSegment ${MXNET_PATH})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(DigitSegment  cNNSegment_connector ${LIBS} -lblas -lopenblas)
target_include_directories(DigitSegment PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS DigitSegment FILE DigitSegment.cmake)
