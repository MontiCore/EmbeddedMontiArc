cmake_minimum_required(VERSION 3.15)

project(hardware_emulator)

set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

#Required Packages
# require when using JNI
find_package(JNI REQUIRED)

set(ZYDIS_BUILD_TOOLS OFF)
set(ZYDIS_BUILD_EXAMPLES OFF)
add_subdirectory("${ROOT_DIR}/extern/zydis" zydis)

add_subdirectory("${ROOT_DIR}/extern/pe-parse/pe-parser-library" pe-parse)
add_subdirectory("${ROOT_DIR}/extern/unicorn" unicorn)

# Do not use as CMake sub-project => Keep flexibility in what to include and compile
set(SHARED_CPP "${ROOT_DIR}/extern/shared_cpp")







message("CMakeLists.txt of hard_emu")



# HARDWARE EMULATOR
add_library(hardware_emulator STATIC)

target_include_directories(hardware_emulator PUBLIC src "../extern/json" "${SHARED_CPP}")
target_link_libraries(hardware_emulator PUBLIC Zydis unicorn_static pe-parser-library)
if (CMAKE_HOST_WIN32)
  target_link_libraries(hardware_emulator PUBLIC Dbghelp Shlwapi)
elseif(CMAKE_HOST_UNIX)
  target_link_libraries(hardware_emulator PUBLIC dl stdc++fs)
endif()
target_compile_features(hardware_emulator PUBLIC cxx_std_17)
target_compile_definitions(hardware_emulator PUBLIC _CRT_SECURE_NO_WARNINGS NO_ERR_OUT)
if ( CMAKE_COMPILER_IS_GNUCC )
  target_compile_options(hardware_emulator PRIVATE
    -fPIC
    #-fpermissive
    #-static-libgcc
    #-static-libstdc++
  )
endif()

set(hardware_emulator_SOURCES
src/simulator/software_simulator_manager.h
src/simulator/software_simulator_manager.cpp
src/simulator/software_simulator.h
src/simulator/software_simulator.cpp
src/simulator/hardware_emulator.h
src/simulator/hardware_emulator.cpp
src/simulator/direct_software_simulator.h
src/simulator/direct_software_simulator.cpp

src/utility/debug.h
src/utility/debug.cpp
src/utility/utility.h
src/utility/utility.cpp
src/utility/dll_interface.h
src/utility/dll_interface.cpp

src/computer/instruction_time.h
src/computer/instruction_time.cpp
src/computer/computer.h
src/computer/computer.cpp
src/computer/computer_layout.h
src/computer/memory.h
src/computer/memory.cpp
src/computer/registers.h
src/computer/registers.cpp
src/computer/system_calls.h
src/computer/system_calls.cpp
src/computer/symbols.h
src/computer/symbols.cpp
src/computer/function_calling.h
src/computer/function_calling.cpp
src/computer/os.h
src/computer/os.cpp
src/computer/caching.h
src/computer/caching.cpp

src/os_windows/dll_loader.h
src/os_windows/dll_loader.cpp
src/os_windows/os_windows.h
src/os_windows/os_windows.cpp
src/os_windows/windows_system_calls.h
src/os_windows/windows_system_calls.cpp
src/os_windows/windows_fast_call.h

src/os_linux/elf.h
src/os_linux/elf.cpp
src/os_linux/elf_loader.h
src/os_linux/elf_loader.cpp
src/os_linux/os_linux.h
src/os_linux/os_linux.cpp
src/os_linux/linux_system_calls.h
src/os_linux/linux_system_calls.cpp
src/os_linux/linux_fast_call.h

src/timetable/skylake.table
)
set(hardware_emulator_EXTERNAL_SOURCES
  ${SHARED_CPP}/buffer.cpp
  ${SHARED_CPP}/program_interface.cpp
  ${SHARED_CPP}/json.cpp
  ${SHARED_CPP}/printf.cpp
  ${SHARED_CPP}/err_out.cpp
)
target_sources(hardware_emulator PRIVATE ${hardware_emulator_SOURCES} ${hardware_emulator_EXTERNAL_SOURCES})















# HARDWARE EMULATOR LIB (Used as Native Java Library)
add_library(hardware_emulator_lib SHARED)

target_link_libraries(hardware_emulator_lib PRIVATE hardware_emulator "${JAVA_AWT_LIBRARY}" "${JAVA_JVM_LIBRARY}")
target_include_directories(hardware_emulator_lib PRIVATE "${JAVA_INCLUDE_PATH}" "${JAVA_INCLUDE_PATH2}")
set_target_properties(hardware_emulator_lib PROPERTIES PREFIX "")
string(TOLOWER "${CMAKE_SYSTEM_NAME}" SYS_NAME)
install(
  TARGETS hardware_emulator_lib
  CONFIGURATIONS Release
  RUNTIME
    DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/../src/main/resources/${SYS_NAME}"
  LIBRARY 
    DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/../src/main/resources/${SYS_NAME}"
)
set(hardware_emulator_lib_SOURCES
  src/lib/de_rwth_montisim_hardware_emulator_CppBridge.h
  src/lib/jni_interface.h
  src/lib/jni_interface.cpp
  src/lib/cpp_bridge.cpp
)
target_sources(hardware_emulator_lib PRIVATE ${hardware_emulator_lib_SOURCES})

if ( CMAKE_COMPILER_IS_GNUCC )
  target_compile_options(hardware_emulator_lib PRIVATE
    -fPIC
    #-fpermissive
    #-static-libgcc
    #-static-libstdc++
  )
endif()








# HARDWARE EMULATOR TEST
add_executable(hardware_emulator_test)

target_link_libraries(hardware_emulator_test PRIVATE hardware_emulator)
set_target_properties(hardware_emulator_test PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set(hardware_emulator_test_SOURCES
  src/test/main.cpp
  src/test/tests.cpp
  src/test/tests.h
)
target_sources(hardware_emulator_test PRIVATE ${hardware_emulator_test_SOURCES})




# HARDWARE EMULATOR SERVER
#[[
add_executable(hardware_emulator_server)

target_link_libraries(hardware_emulator_server PRIVATE hardware_emulator)
set_target_properties(hardware_emulator_server PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set(hardware_emulator_server_SOURCES
  src/server/server.cpp
)
set(hardware_emulator_server_EXTERN_SOURCES
  ${SHARED_CPP}/network.cpp
)
target_sources(hardware_emulator_server PRIVATE ${hardware_emulator_server_SOURCES} ${hardware_emulator_server_EXTERN_SOURCES})
]]








source_group(
  TREE "${CMAKE_CURRENT_SOURCE_DIR}/src"
  PREFIX "src"
  FILES ${hardware_emulator_SOURCES} ${hardware_emulator_lib_SOURCES} ${hardware_emulator_test_SOURCES} ${hardware_emulator_server_SOURCES}
)


