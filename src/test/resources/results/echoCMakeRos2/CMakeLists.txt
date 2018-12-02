cmake_minimum_required(VERSION 3.5)
project (RosAdapter_tests_a_compA)

find_package(automated_driving_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_library(RosAdapter_tests_a_compA RosAdapter_tests_a_compA.h)
set_target_properties(RosAdapter_tests_a_compA PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(RosAdapter_tests_a_compA tests_a_compA ${automated_driving_msgs_LIBRARIES} ${rclcpp_LIBRARIES} ${rosgraph_msgs_LIBRARIES})
target_include_directories(RosAdapter_tests_a_compA PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${automated_driving_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${rosgraph_msgs_INCLUDE_DIRS})


export(TARGETS RosAdapter_tests_a_compA FILE RosAdapter_tests_a_compA.cmake)