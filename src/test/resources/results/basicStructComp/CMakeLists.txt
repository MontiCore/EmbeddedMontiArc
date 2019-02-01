cmake_minimum_required(VERSION 3.5)
project (RosAdapter_tests_msg_basicStructComp)
set (CMAKE_CXX_STANDARD 14)
set (AMENT_CMAKE_UNINSTALL_TARGET FALSE)

find_package(roscpp REQUIRED)

add_library(RosAdapter_tests_msg_basicStructComp RosAdapter_tests_msg_basicStructComp.cpp)
set_target_properties(RosAdapter_tests_msg_basicStructComp PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(RosAdapter_tests_msg_basicStructComp tests_msg_basicStructComp IAdapter_tests_msg_basicStructComp ${roscpp_LIBRARIES})
target_include_directories(RosAdapter_tests_msg_basicStructComp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${roscpp_INCLUDE_DIRS})


export(TARGETS RosAdapter_tests_msg_basicStructComp FILE RosAdapter_tests_msg_basicStructComp.cmake)

# Generate .h files from .msg files
add_custom_target(gen_RosAdapter_tests_msg_basicStructComp_struct_msgs_tests_structs_BasicStruct ALL
COMMAND python /opt/ros/kinetic/lib/gencpp/gen_cpp.py ${CMAKE_CURRENT_SOURCE_DIR}/struct_msgs/tests_structs_BasicStruct.msg -Istruct_msgs:${CMAKE_CURRENT_SOURCE_DIR}/struct_msgs -p struct_msgs -o ${CMAKE_CURRENT_SOURCE_DIR}/struct_msgs -e /opt/ros/kinetic/share/gencpp/)

add_dependencies(RosAdapter_tests_msg_basicStructComp gen_RosAdapter_tests_msg_basicStructComp_struct_msgs_tests_structs_BasicStruct)