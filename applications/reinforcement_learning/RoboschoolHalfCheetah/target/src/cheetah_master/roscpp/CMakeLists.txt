cmake_minimum_required(VERSION 3.5)
project (RosAdapter_cheetah_master)
set (CMAKE_CXX_STANDARD 14)
set (AMENT_CMAKE_UNINSTALL_TARGET FALSE)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(RosAdapter_cheetah_master RosAdapter_cheetah_master.cpp)

list(APPEND LIBRARIES cheetah_master)
list(APPEND LIBRARIES IAdapter_cheetah_master)
list(APPEND LIBRARIES ${roscpp_LIBRARIES})
list(APPEND LIBRARIES ${std_msgs_LIBRARIES})
target_link_libraries(RosAdapter_cheetah_master ${LIBRARIES})

list(APPEND INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND INCLUDE_DIRS ${roscpp_INCLUDE_DIRS})
list(APPEND INCLUDE_DIRS ${std_msgs_INCLUDE_DIRS})
target_include_directories(RosAdapter_cheetah_master PUBLIC ${INCLUDE_DIRS})

export(TARGETS RosAdapter_cheetah_master FILE RosAdapter_cheetah_master.cmake)