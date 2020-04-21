cmake_minimum_required(VERSION 3.5)
project(BallTracking CXX)
set(CMAKE_CXX_STANDARD 11)


add_executable(BallTracking BallTracking.cpp)

set_target_properties(BallTracking PROPERTIES LINKER_LANGUAGE CXX)
find_package(OpenCV REQUIRED)

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(BallTracking ba_balltracking ${LIBS} -lblas -lopenblas)
target_include_directories(BallTracking PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
export(TARGETS BallTracking FILE BallTracking.cmake)
