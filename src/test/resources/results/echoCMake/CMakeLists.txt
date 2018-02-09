cmake_minimum_required(VERSION 3.5)
project (tests_a_compA_RosWrapper)

find_package(automated_driving_msgs REQUIRED)
find_package(roscpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_library(tests_a_compA_RosWrapper tests_a_compA_RosWrapper.h)
set_target_properties(tests_a_compA_RosWrapper PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(tests_a_compA_RosWrapper tests_a_compA ${automated_driving_msgs_LIBRARIES} ${roscpp_LIBRARIES} ${rosgraph_msgs_LIBRARIES})
target_include_directories(tests_a_compA_RosWrapper PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${automated_driving_msgs_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS} ${rosgraph_msgs_INCLUDE_DIRS})

export(TARGETS tests_a_compA_RosWrapper FILE tests_a_compA_RosWrapper.cmake)