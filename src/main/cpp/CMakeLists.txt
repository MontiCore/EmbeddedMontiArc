cmake_minimum_required(VERSION 3.5)
project(DigitClassifier CXX)
set(CMAKE_CXX_STANDARD 11)



add_executable(DigitClassifier DigitClassifier.cpp)
set_target_properties(DigitClassifier PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(DigitClassifier  mnist_mnistClassifier ${LIBS} -lblas -lopenblas)
target_include_directories(DigitClassifier PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS DigitClassifier FILE DigitClassifier.cmake)
