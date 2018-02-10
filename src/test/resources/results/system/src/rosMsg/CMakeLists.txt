cmake_minimum_required(VERSION 3.5)
project (struct_msgs)

find_package(genmsg REQUIRED)

FILE(GLOB MSG_FILES_RAW RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ../*/*/struct_msgs/*.msg)

#generate struct_msgs iff .msg files where found
if(MSG_FILES_RAW)

    #filter: add each struct msg only once (distinct by filename without path)
    foreach(CUR_MSG_FILE ${MSG_FILES_RAW})
        get_filename_component(TMP_MSG_NAME ${CUR_MSG_FILE} NAME)
        IF(NOT MSG_DEFINED_${TMP_MSG_NAME})
            get_filename_component(DIR_${TMP_MSG_NAME} ${CUR_MSG_FILE} DIRECTORY)
            add_message_files(DIRECTORY ${DIR_${TMP_MSG_NAME}} FILES ${TMP_MSG_NAME})
            SET(MSG_DEFINED_${TMP_MSG_NAME} TRUE)
        ENDIF()
    endforeach(CUR_MSG_FILE)

    generate_messages()
    #export the include_dirs, so that other subprojects can use it
    set(struct_msgs_INCLUDE_DIRS ${struct_msgs_INCLUDE_DIRS} PARENT_SCOPE)
endif()