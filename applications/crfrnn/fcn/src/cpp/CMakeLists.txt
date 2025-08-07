cmake_minimum_required(VERSION 3.5)
project(DemoSemSeg CXX)
set(CMAKE_CXX_STANDARD 11)

option(MXNET_PATH "Path to libmxnet.so")


add_executable(DemoSemSeg DemoSemSeg.cpp)

set_target_properties(DemoSemSeg PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
target_link_libraries(DemoSemSeg ${MXNET_PATH})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(DemoSemSeg imageSegmentation_segmentation ${LIBS} -lblas -lopenblas)
target_include_directories(DemoSemSeg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS DemoSemSeg FILE DemoSemSeg.cmake)
