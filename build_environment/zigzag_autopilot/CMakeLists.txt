cmake_minimum_required(VERSION 3.15)

project(zigzag_autopilot)

message(STATUS "SHARED_CPP_DIR=${SHARED_CPP_DIR}")
if(SHARED_CPP_DIR)
    message( STATUS "Found SHARED_CPP directory: ${SHARED_CPP_DIR}")
else()
    message(STATUS "SHARED_CPP_DIR=${SHARED_CPP_DIR}")
    message( FATAL_ERROR "This project requires the SHARED_CPP project. Specify using -DSHARED_CPP_DIR=path/to/shared_cpp" )
endif()


list(APPEND AUTOPILOT_FILES
    src/autopilot.h
    src/autopilot.cpp

    ${SHARED_CPP_DIR}/json.h
    ${SHARED_CPP_DIR}/json.cpp
    ${SHARED_CPP_DIR}/buffer.h
    ${SHARED_CPP_DIR}/buffer.cpp
    ${SHARED_CPP_DIR}/printf.h
    ${SHARED_CPP_DIR}/printf.cpp
    ${SHARED_CPP_DIR}/utils.h
    ${SHARED_CPP_DIR}/err_out.h
    ${SHARED_CPP_DIR}/err_out.cpp
)


list(APPEND LIB_FILES
    ${SHARED_CPP_DIR}/library_interface.h
    src/library_interface.cpp
)

list(APPEND SERVER_ADAPTER_FILES
    src/server_adapter.h
    src/server_adapter.cpp

    ${SHARED_CPP_DIR}/network.h
    ${SHARED_CPP_DIR}/network.cpp
    ${SHARED_CPP_DIR}/tcp_protocol.h
    ${SHARED_CPP_DIR}/standard_err_out.h
    ${SHARED_CPP_DIR}/standard_err_out.cpp
)



add_executable(zigzag_autopilot_server ${AUTOPILOT_FILES} ${SERVER_ADAPTER_FILES})
target_include_directories(zigzag_autopilot_server PUBLIC ${SHARED_CPP_DIR})
target_compile_options(zigzag_autopilot_server PUBLIC -std=c++11)
if (CMAKE_HOST_WIN32)
    target_link_libraries(zigzag_autopilot_server PUBLIC ws2_32)
endif()


add_library(zigzag_autopilot_lib SHARED ${AUTOPILOT_FILES} ${LIB_FILES})
target_link_libraries(zigzag_autopilot_lib PUBLIC -static-libgcc -static-libstdc++)
target_include_directories(zigzag_autopilot_lib PUBLIC ${SHARED_CPP_DIR})
set_target_properties(zigzag_autopilot_lib PROPERTIES LINKER_LANGUAGE CXX POSITION_INDEPENDENT_CODE ON)
set_target_properties(zigzag_autopilot_lib PROPERTIES PREFIX "")
target_compile_features(zigzag_autopilot_lib PUBLIC cxx_std_11)