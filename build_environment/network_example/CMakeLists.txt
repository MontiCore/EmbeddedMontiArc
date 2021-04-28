cmake_minimum_required(VERSION 3.5)

project(network_example)

set(SHARED_CPP_DIR $ENV{SHARED_CPP_PATH})
set(LIBDDC_DIR $ENV{LIBDDC_PATH})

if (LIBDDC_DIR)
    message("Using libddc")
    set(USING_DDC TRUE)
    add_subdirectory(${LIBDDC_DIR} libddc)
endif()


list(APPEND AUTOPILOT_FILES
    src/autopilot.h
    src/autopilot.cpp
    src/program_adapter.h
    src/program_adapter.cpp

    ${SHARED_CPP_DIR}/json.h
    ${SHARED_CPP_DIR}/json.cpp
    ${SHARED_CPP_DIR}/printf.h
    ${SHARED_CPP_DIR}/printf.cpp
    ${SHARED_CPP_DIR}/utils.h
    ${SHARED_CPP_DIR}/program_interface.h
    ${SHARED_CPP_DIR}/program_interface.cpp
    ${SHARED_CPP_DIR}/network.h
    ${SHARED_CPP_DIR}/network.cpp
    ${SHARED_CPP_DIR}/ddc_interface.h
    ${SHARED_CPP_DIR}/tcp_protocol.h
)
if (USING_DDC)
    list(APPEND AUTOPILOT_FILES
        src/ddc_mode.h
        src/ddc_mode.cpp
    )
endif()



add_executable(network_example ${AUTOPILOT_FILES})
target_include_directories(network_example PUBLIC ${SHARED_CPP_DIR})
target_compile_options(network_example PUBLIC -std=c++11)
if (USING_DDC)
    target_link_libraries(network_example PUBLIC ddc_load)
    target_compile_definitions(network_example PUBLIC USE_DDC)
endif()