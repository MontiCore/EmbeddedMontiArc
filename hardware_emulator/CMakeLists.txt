cmake_minimum_required(VERSION 3.7)
project(hardware-emulator)

set(AUTOPILO_ADAPTER_NAME "AutopilotAdapter")

list(APPEND HARDWARE_EMU_SOURCEFILES
  include/hardware-emulator/debug.h
  include/hardware-emulator/dll_interface.h
  include/hardware-emulator/autopilot_interface.h
  include/hardware-emulator/method_calling.h
  include/hardware-emulator/tests.h
  include/hardware-emulator/utility.h
  include/hardware-emulator/instruction_time.h
  
  include/hardware-emulator/computer/computer.h
  include/hardware-emulator/computer/computer_layout.h
  include/hardware-emulator/computer/memory.h
  include/hardware-emulator/computer/registers.h
  include/hardware-emulator/computer/system_calls.h
  
  include/hardware-emulator/os_windows/dll_loader.h
  include/hardware-emulator/os_windows/os_windows.h
  include/hardware-emulator/os_windows/windows_calls.h
  
  include/hardware-emulator/jni/jni_emulator.h
  
  src/debug.cpp
  src/dll_interface.cpp
  src/autopilot_interface.cpp
  src/method_calling.cpp
  src/tests.cpp
  src/utility.cpp
  src/instruction_time.cpp
  
  src/computer/computer.cpp
  src/computer/memory.cpp
  src/computer/registers.cpp
  src/computer/system_calls.cpp
  
  src/os_windows/dll_loader.cpp
  src/os_windows/os_windows.cpp
  src/os_windows/windows_calls.cpp
  
  src/jni/jni_emulator.cpp
)

list(APPEND HARDWARE_EMU_TEST_SOURCEFILES
    src/main.cpp
)

list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_SOURCEFILES
  include/hardware-emulator/jni/simulator_integration_AutopilotAdapter.h
  src/jni/AutopilotAdapter.cpp
)

foreach(source IN LISTS HARDWARE_EMU_SOURCEFILES)
    string(REPLACE "include/hardware-emulator/" "" trunc_source "${source}")
    string(REPLACE "src/" "" trunc_source2 "${trunc_source}")
    get_filename_component(source_path "${trunc_source2}" PATH)
    string(REPLACE "/" "\\" source_path_msvc "${source_path}")
    source_group("${source_path_msvc}" FILES "${source}")
endforeach()

foreach(source IN LISTS HARDWARE_EMU_TEST_SOURCEFILES)
    string(REPLACE "include/hardware-emulator/" "" trunc_source "${source}")
    string(REPLACE "src/" "" trunc_source2 "${trunc_source}")
    get_filename_component(source_path "${trunc_source2}" PATH)
    string(REPLACE "/" "\\" source_path_msvc "${source_path}")
    source_group("${source_path_msvc}" FILES "${source}")
endforeach()

foreach(source IN LISTS HARDWARE_EMU_AUTOPILOT_ADAPTER_SOURCEFILES)
    string(REPLACE "include/hardware-emulator/" "" trunc_source "${source}")
    string(REPLACE "src/" "" trunc_source2 "${trunc_source}")
    get_filename_component(source_path "${trunc_source2}" PATH)
    string(REPLACE "/" "\\" source_path_msvc "${source_path}")
    source_group("${source_path_msvc}" FILES "${source}")
endforeach()

list(APPEND HARDWARE_EMU_INCLUDE_DIRS
  include/hardware-emulator
  ../unicorn/include
  ../pe-parse/pe-parser-library/include
  ../zydis/include
  ../zydis/dependencies/zycore/include
  ../zydis/msvc
  ../jdk/include
  ../jdk/include/win32
)

list(APPEND HARDWARE_EMU_LIB_DIRS
  ../Zydis/build
  ../unicorn/build
  ../unicorn/build/qemu
  ../pe-parse/build/pe-parser-library
)


list(APPEND HARDWARE_EMU_LIBRARIES
  Zydis
  unicorn
  pe-parser-library
  x86_64-softmmu
  Dbghelp
)

list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_INCLUDE_DIRS
)

list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_LIB_DIRS
  ../jdk/lib
)

list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_LIBRARIES
    jawt
    jvm
)

LINK_DIRECTORIES(${PROJECT_NAME}-test ${HARDWARE_EMU_LIB_DIRS})
add_executable(${PROJECT_NAME}-test ${HARDWARE_EMU_SOURCEFILES} ${HARDWARE_EMU_TEST_SOURCEFILES})
target_include_directories(${PROJECT_NAME}-test PUBLIC ${HARDWARE_EMU_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-test ${HARDWARE_EMU_LIBRARIES})

add_definitions(
    /std:c++17
    /DZYDIS_STATIC_DEFINE
)

set_target_properties(${PROJECT_NAME}-test PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

LINK_DIRECTORIES(${AUTOPILO_ADAPTER_NAME} ${HARDWARE_EMU_LIB_DIRS} ${HARDWARE_EMU_AUTOPILOT_ADAPTER_LIB_DIRS})
add_library(${AUTOPILO_ADAPTER_NAME} SHARED ${HARDWARE_EMU_SOURCEFILES} ${HARDWARE_EMU_AUTOPILOT_ADAPTER_SOURCEFILES})
target_include_directories(${AUTOPILO_ADAPTER_NAME} PUBLIC ${HARDWARE_EMU_INCLUDE_DIRS} ${HARDWARE_EMU_AUTOPILOT_ADAPTER_INCLUDE_DIRS} )
target_link_libraries(${AUTOPILO_ADAPTER_NAME} ${HARDWARE_EMU_LIBRARIES} ${HARDWARE_EMU_AUTOPILOT_ADAPTER_LIBRARIES})

add_definitions(
    /std:c++17
    /DZYDIS_STATIC_DEFINE
)
