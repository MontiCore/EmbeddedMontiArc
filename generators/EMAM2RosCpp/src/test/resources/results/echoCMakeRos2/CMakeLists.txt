cmake_minimum_required(VERSION 3.5)
project (RosAdapter_tests_a_compB)
set (CMAKE_CXX_STANDARD 14)
set (AMENT_CMAKE_UNINSTALL_TARGET FALSE)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(RosAdapter_tests_a_compB RosAdapter_tests_a_compB.cpp)

list(APPEND LIBRARIES tests_a_compB)
list(APPEND LIBRARIES IAdapter_tests_a_compB)
list(APPEND LIBRARIES ${rclcpp_LIBRARIES})
list(APPEND LIBRARIES ${std_msgs_LIBRARIES})
target_link_libraries(RosAdapter_tests_a_compB ${LIBRARIES})

list(APPEND INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS})
list(APPEND INCLUDE_DIRS ${std_msgs_INCLUDE_DIRS})
target_include_directories(RosAdapter_tests_a_compB PUBLIC ${INCLUDE_DIRS})

export(TARGETS RosAdapter_tests_a_compB FILE RosAdapter_tests_a_compB.cmake)
