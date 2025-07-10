cmake_minimum_required(VERSION 2.8)
project( steeringAnglePredictor )
add_executable( steeringAnglePredictor steeringAnglePredictor.cpp )

find_package( OpenCV REQUIRED )
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} -lblas -lopenblas)
set(LIBS ${LIBS} ${OpenCV_LIBS})
target_link_libraries(steeringAnglePredictor endtoend_nvidia ${LIBS})
target_include_directories(steeringAnglePredictor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


target_link_libraries( steeringAnglePredictor ${OpenCV_LIBS} )
