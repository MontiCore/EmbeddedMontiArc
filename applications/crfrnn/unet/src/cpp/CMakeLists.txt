cmake_minimum_required(VERSION 3.5)
project(DemoUnet CXX)
set(CMAKE_CXX_STANDARD 11)

option(MXNET_PATH "Path to libmxnet.so")


add_executable(DemoUnet DemoUnet.cpp)

set_target_properties(DemoUnet PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
target_link_libraries(DemoUnet ${MXNET_PATH})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(DemoUnet imageSegmentation_segmentation ${LIBS} -lblas -lopenblas)
target_include_directories(DemoUnet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS DemoUnet FILE DemoUnet.cmake)
