cmake_minimum_required(VERSION 3.5)
project(Translator)

set(CMAKE_CXX_STANDARD 14)

add_executable(Translator Translator.cpp)

target_link_libraries(Translator ${MXNET_PATH})
target_link_libraries(Translator translator_rNNsearch ${LIBS} -lblas -lopenblas)
target_include_directories(Translator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
