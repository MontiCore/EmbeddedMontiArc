cmake_minimum_required(VERSION 3.5)
project (RosAdapter_de_rwth_montisim_agent_master)
set (CMAKE_CXX_STANDARD 14)
set (AMENT_CMAKE_UNINSTALL_TARGET FALSE)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(RosAdapter_de_rwth_montisim_agent_master RosAdapter_de_rwth_montisim_agent_master.cpp)

list(APPEND LIBRARIES de_rwth_montisim_agent_master)
list(APPEND LIBRARIES IAdapter_de_rwth_montisim_agent_master)
list(APPEND LIBRARIES ${roscpp_LIBRARIES})
list(APPEND LIBRARIES ${std_msgs_LIBRARIES})
target_link_libraries(RosAdapter_de_rwth_montisim_agent_master ${LIBRARIES})

list(APPEND INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND INCLUDE_DIRS ${roscpp_INCLUDE_DIRS})
list(APPEND INCLUDE_DIRS ${std_msgs_INCLUDE_DIRS})
target_include_directories(RosAdapter_de_rwth_montisim_agent_master PUBLIC ${INCLUDE_DIRS})

export(TARGETS RosAdapter_de_rwth_montisim_agent_master FILE RosAdapter_de_rwth_montisim_agent_master.cmake)
