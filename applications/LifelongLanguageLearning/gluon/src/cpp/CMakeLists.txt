cmake_minimum_required(VERSION 3.5)
project(LifelongLanguageLearning CXX)
set(CMAKE_CXX_STANDARD 11)

option(MXNET_PATH "Path to libmxnet.so")


add_executable(LifelongLanguageLearning LifelongLanguageLearning.cpp)

set_target_properties(LifelongLanguageLearning PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(LifelongLanguageLearning ${MXNET_PATH})
target_link_libraries(LifelongLanguageLearning lifelongLanguageLearning_connector ${LIBS} -lblas -lopenblas)
target_include_directories(LifelongLanguageLearning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS LifelongLanguageLearning FILE LifelongLanguageLearning.cmake)
