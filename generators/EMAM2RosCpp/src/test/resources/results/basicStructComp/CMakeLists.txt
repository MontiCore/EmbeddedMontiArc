cmake_minimum_required(VERSION 3.5)
project (RosAdapter_tests_msg_basicStructComp)
set (CMAKE_CXX_STANDARD 14)
set (AMENT_CMAKE_UNINSTALL_TARGET FALSE)

find_package(roscpp REQUIRED)

add_library(RosAdapter_tests_msg_basicStructComp RosAdapter_tests_msg_basicStructComp.cpp)

list(APPEND LIBRARIES tests_msg_basicStructComp)
list(APPEND LIBRARIES IAdapter_tests_msg_basicStructComp)
list(APPEND LIBRARIES ${roscpp_LIBRARIES})
list(APPEND LIBRARIES ${struct_msgs_LIBRARIES})
target_link_libraries(RosAdapter_tests_msg_basicStructComp ${LIBRARIES})

list(APPEND INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND INCLUDE_DIRS ${roscpp_INCLUDE_DIRS})
list(APPEND INCLUDE_DIRS ${struct_msgs_INCLUDE_DIRS})
target_include_directories(RosAdapter_tests_msg_basicStructComp PUBLIC ${INCLUDE_DIRS})

export(TARGETS RosAdapter_tests_msg_basicStructComp FILE RosAdapter_tests_msg_basicStructComp.cmake)
