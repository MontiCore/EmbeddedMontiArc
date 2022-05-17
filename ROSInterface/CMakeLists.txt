cmake_minimum_required(VERSION 3.5)
project(ROSInterface)
set (CMAKE_CXX_STANDARD 14)
set (AMENT_CMAKE_UNINSTALL_TARGET FALSE)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(ROSInterface SHARED src/ROSInterface.cpp)

list(APPEND LIBRARIES ${roscpp_LIBRARIES})
list(APPEND LIBRARIES ${std_msgs_LIBRARIES})
target_link_libraries(ROSInterface ${LIBRARIES})

list(APPEND INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND INCLUDE_DIRS ${roscpp_INCLUDE_DIRS})
list(APPEND INCLUDE_DIRS ${std_msgs_INCLUDE_DIRS})
list(APPEND INCLUDE_DIRS /usr/lib/jvm/java-8-openjdk-amd64/include)
list(APPEND INCLUDE_DIRS /usr/lib/jvm/java-8-openjdk-amd64/include/linux)
target_include_directories(ROSInterface PUBLIC ${INCLUDE_DIRS})

