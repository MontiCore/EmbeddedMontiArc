cmake_minimum_required(VERSION 3.5)
project(TestMNIST CXX)
set(CMAKE_CXX_STANDARD 11)

option(MXNET_PATH "Path to libmxnet.so")


add_executable(TestMNIST TestMNIST.cpp)

set_target_properties(TestMNIST PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
target_link_libraries(TestMNIST ${MXNET_PATH})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(TestMNIST mnist_mnistClassifier ${LIBS} -lblas -lopenblas)
target_include_directories(TestMNIST PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS TestMNIST FILE TestMNIST.cmake)


