cmake_minimum_required(VERSION 3.5)
project(FlowNetS CXX)
set(CMAKE_CXX_STANDARD 11)

add_executable(FlowNetS FlowNetS.cpp)

set_target_properties(FlowNetS PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} -lblas -lopenblas)
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(FlowNetS flowNetS_network ${LIBS})
target_include_directories(FlowNetS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS FlowNetS FILE FlowNetS.cmake)
