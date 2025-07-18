cmake_minimum_required(VERSION 3.14)
project(agent_direct)

set(CMAKE_CXX_STANDARD 14)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

add_subdirectory(torcs_agent_torcsAgent)

add_executable(agent_direct main.cpp DirectRosAgent.cpp DirectRosAgent.h)

target_link_libraries(agent_direct ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} torcs_agent_torcsAgent)
target_include_directories(agent_direct PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
