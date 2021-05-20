cmake_minimum_required(VERSION 3.15)

project(zigzagap)

set(SHARED_CPP_DIR "../../extern/shared_cpp")

add_library(cppautopilot_zigzag_lib SHARED "autopilot.cpp" "library_interface.cpp" "${SHARED_CPP_DIR}/buffer.cpp" "${SHARED_CPP_DIR}/json.cpp" "${SHARED_CPP_DIR}/printf.cpp")
target_include_directories(cppautopilot_zigzag_lib PUBLIC ${SHARED_CPP_DIR})

add_executable(cppautopilot_zigzag_server "autopilot.cpp" "server_adapter.cpp" "${SHARED_CPP_DIR}/buffer.cpp" "${SHARED_CPP_DIR}/json.cpp" "${SHARED_CPP_DIR}/network.cpp" "${SHARED_CPP_DIR}/printf.cpp")
target_include_directories(cppautopilot_zigzag_server PUBLIC ${SHARED_CPP_DIR})