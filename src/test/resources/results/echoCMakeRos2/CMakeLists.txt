cmake_minimum_required(VERSION 3.5)
project (RosAdapter_tests_a_compA)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(RosAdapter_tests_a_compA RosAdapter_tests_a_compA.cxx)
set_target_properties(RosAdapter_tests_a_compA PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(RosAdapter_tests_a_compA tests_a_compA ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES} IAdapter_tests_a_compA)
target_include_directories(RosAdapter_tests_a_compA PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${std_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})


export(TARGETS RosAdapter_tests_a_compA FILE RosAdapter_tests_a_compA.cmake)