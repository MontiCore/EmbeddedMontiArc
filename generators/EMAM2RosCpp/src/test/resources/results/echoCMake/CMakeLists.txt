cmake_minimum_required(VERSION 3.5)
project (RosAdapter_tests_a_compA)
set (CMAKE_CXX_STANDARD 14)
set (AMENT_CMAKE_UNINSTALL_TARGET FALSE)

find_package(automated_driving_msgs REQUIRED)
find_package(roscpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_library(RosAdapter_tests_a_compA RosAdapter_tests_a_compA.cpp)

list(APPEND LIBRARIES tests_a_compA)
list(APPEND LIBRARIES IAdapter_tests_a_compA)
list(APPEND LIBRARIES ${automated_driving_msgs_LIBRARIES})
list(APPEND LIBRARIES ${roscpp_LIBRARIES})
list(APPEND LIBRARIES ${rosgraph_msgs_LIBRARIES})
target_link_libraries(RosAdapter_tests_a_compA ${LIBRARIES})

list(APPEND INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND INCLUDE_DIRS ${automated_driving_msgs_INCLUDE_DIRS})
list(APPEND INCLUDE_DIRS ${roscpp_INCLUDE_DIRS})
list(APPEND INCLUDE_DIRS ${rosgraph_msgs_INCLUDE_DIRS})
target_include_directories(RosAdapter_tests_a_compA PUBLIC ${INCLUDE_DIRS})

export(TARGETS RosAdapter_tests_a_compA FILE RosAdapter_tests_a_compA.cmake)
