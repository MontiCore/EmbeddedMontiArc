cmake_minimum_required(VERSION 3.5)
project(ShowAttendTell CXX)

set(CMAKE_CXX_STANDARD 11)

add_executable(ShowAttendTell ShowAttendTell.cpp)

set_target_properties(ShowAttendTell PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)
target_link_libraries(ShowAttendTell ${MXNET_PATH})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(ShowAttendTell showAttendTell_main ${LIBS} -lblas -lopenblas)
target_include_directories(ShowAttendTell PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS ShowAttendTell FILE ShowAttendTell.cmake)

