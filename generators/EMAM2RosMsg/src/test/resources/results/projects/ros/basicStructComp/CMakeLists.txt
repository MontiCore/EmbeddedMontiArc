# generated by ros.MsgGen.CMakeLists.ftl
cmake_minimum_required(VERSION 2.8.12)
project(struct_msgs)
find_package(genmsg REQUIRED)

add_message_files(FILES structs_BasicStruct.msg)
set(CATKIN_MESSAGE_GENERATORS gencpp)
generate_messages()
set(struct_msgs_INCLUDE_DIRS ${struct_msgs_INCLUDE_DIRS} PARENT_SCOPE)
