cmake_minimum_required(VERSION 3.5)
project (RosAdapter_tests_a_compB)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(RosAdapter_tests_a_compB RosAdapter_tests_a_compB.cpp)
set_target_properties(RosAdapter_tests_a_compB PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(RosAdapter_tests_a_compB tests_a_compB IAdapter_tests_a_compB ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
target_include_directories(RosAdapter_tests_a_compB PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})


export(TARGETS RosAdapter_tests_a_compB FILE RosAdapter_tests_a_compB.cmake)