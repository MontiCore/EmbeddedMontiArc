cmake_minimum_required(VERSION 3.5)
project(ThreeDGan CXX)
set(CMAKE_CXX_STANDARD 11)

option(MXNET_PATH "Path to libmxnet.so")


add_executable(ThreeDGan ThreeDGan.cpp)

set_target_properties(ThreeDGan PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
target_link_libraries(ThreeDGan ${MXNET_PATH})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(ThreeDGan  threedgan_connector ${LIBS} -lblas -lopenblas)
target_include_directories(ThreeDGan PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS ThreeDGan FILE ThreeDGan.cmake)
