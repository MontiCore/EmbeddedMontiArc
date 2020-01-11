cmake_minimum_required(VERSION 3.5)
project(Translator CXX)

set(CMAKE_CXX_STANDARD 11)

add_executable(Translator Translator.cpp)

set_target_properties(Translator PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
target_link_libraries(Translator ${MXNET_PATH})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(Translator translator_main ${LIBS} -lblas -lopenblas)
target_include_directories(Translator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS Translator FILE Translator.cmake)

