cmake_minimum_required(VERSION 3.15)

project(hardware_emulator)

set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

#Required Packages
find_package(JNI REQUIRED)


set(ZYDIS_BUILD_TOOLS OFF)
set(ZYDIS_BUILD_EXAMPLES OFF)
add_subdirectory("${ROOT_DIR}/extern/zydis" zydis)

add_subdirectory("${ROOT_DIR}/extern/pe-parse/pe-parser-library" pe-parse)
add_subdirectory("${ROOT_DIR}/extern/unicorn" unicorn)










# HARDWARE EMULATOR
add_library(hardware_emulator STATIC)

target_include_directories(hardware_emulator PUBLIC src ../extern/json)
target_link_libraries(hardware_emulator PUBLIC "Zydis" unicorn_static pe-parser-library)
if (CMAKE_HOST_WIN32)
  target_link_libraries(hardware_emulator PUBLIC Dbghelp Shlwapi)
elseif(CMAKE_HOST_UNIX)
  target_link_libraries(hardware_emulator PUBLIC dl)
endif()
target_compile_features(hardware_emulator PUBLIC cxx_std_17)
target_compile_definitions(hardware_emulator PUBLIC _CRT_SECURE_NO_WARNINGS)


set(hardware_emulator_SOURCES
src/software_simulator_manager.h
src/software_simulator_manager.cpp
src/software_simulator.h
src/software_simulator.cpp
src/hardware_emulator.h
src/hardware_emulator.cpp
src/direct_software_simulator.h
src/direct_software_simulator.cpp

src/utility/debug.h
src/utility/debug.cpp
src/utility/utility.h
src/utility/utility.cpp
src/utility/config.h
src/utility/config.cpp
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
src/os_windows/windows_calls.h
src/os_windows/windows_calls.cpp

src/os_linux/elf.h
src/os_linux/elf.cpp
src/os_linux/elf_loader.h
src/os_linux/elf_loader.cpp
src/os_linux/os_linux.h
src/os_linux/os_linux.cpp
src/os_linux/linux_calls.h
src/os_linux/linux_calls.cpp

src/timetable/skylake.table
)
target_sources(hardware_emulator PRIVATE ${hardware_emulator_SOURCES})
















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








source_group(
  TREE "${CMAKE_CURRENT_SOURCE_DIR}/src"
  PREFIX "src"
  FILES ${hardware_emulator_SOURCES} ${hardware_emulator_lib_SOURCES} ${hardware_emulator_test_SOURCES}
)


# TODO:
# Check how to include the following in Zydis's CMakeLists

# if ( CMAKE_COMPILER_IS_GNUCC )
#   target_include_directories(hardware_emulator PUBLIC ../zydis/gnucc)
# elseif( MSVC )
#   target_include_directories(hardware_emulator PUBLIC ../zydis/msvc)
# endif()

# if ( CMAKE_COMPILER_IS_GNUCC )
#   add_definitions(
#     -fPIC
#     -fpermissive
#     -static-libgcc
#     -static-libstdc++
#   )
# endif()

