cmake_minimum_required(VERSION 3.5)
project(LetterPredictor CXX)
set(CMAKE_CXX_STANDARD 11)

option(MXNET_PATH "Path to libmxnet.so")


add_executable(LetterPredictor LetterPredictor.cpp)

set_target_properties(LetterPredictor PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
target_link_libraries(LetterPredictor ${MXNET_PATH})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(LetterPredictor  letterpred_connector ${LIBS} -lblas -lopenblas)
target_include_directories(LetterPredictor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS LetterPredictor FILE LetterPredictor.cmake)
