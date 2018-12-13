cmake_minimum_required(VERSION 3.7)
project(hardware-emulator)

list(APPEND HARDWARE_EMU_SOURCEFILES
  include/hardware-emulator/debug.h
  include/hardware-emulator/dll_interface.h
  include/hardware-emulator/method_calling.h
  include/hardware-emulator/tests.h
  include/hardware-emulator/utility.h
  
  include/hardware-emulator/computer/computer.h
  include/hardware-emulator/computer/computer_layout.h
  include/hardware-emulator/computer/memory.h
  include/hardware-emulator/computer/registers.h
  include/hardware-emulator/computer/system_calls.h
  
  include/hardware-emulator/os_windows/dll_loader.h
  include/hardware-emulator/os_windows/os_windows.h
  include/hardware-emulator/os_windows/windows_calls.h
  
  src/debug.cpp
  src/dll_interface.cpp
  src/method_calling.cpp
  src/tests.cpp
  src/utility.cpp
  
  src/computer/computer.cpp
  src/computer/memory.cpp
  src/computer/registers.cpp
  src/computer/system_calls.cpp
  
  src/os_windows/dll_loader.cpp
  src/os_windows/os_windows.cpp
  src/os_windows/windows_calls.cpp
  
  src/main.cpp
)

foreach(source IN LISTS HARDWARE_EMU_SOURCEFILES)
    
    string(REPLACE "include/hardware-emulator/" "" trunc_source "${source}")
    string(REPLACE "src/" "" trunc_source2 "${trunc_source}")
    get_filename_component(source_path "${trunc_source2}" PATH)
    string(REPLACE "/" "\\" source_path_msvc "${source_path}")
    source_group("${source_path_msvc}" FILES "${source}")
    #message("Source: ${source} trunc: ${trunc_source2} source_path_msvc: ${source_path_msvc}")
endforeach()

list(APPEND HARDWARE_EMU_INCLUDE_DIRS
  include/hardware-emulator
  ../unicorn/include
  ../pe-parse/pe-parser-library/include
  ../zydis/include
  ../zydis/dependencies/zycore/include
  ../zydis/msvc
)


list(APPEND HARDWARE_EMU_LIB_DIRS
  ../Zydis/build/$<CONFIG>
  ../unicorn/build/$<CONFIG>
  ../unicorn/build/qemu/$<CONFIG>
  ../pe-parse/build/pe-parser-library/$<CONFIG>
)


list(APPEND HARDWARE_EMU_LIBRARIES
  Zydis.lib
  unicorn.lib
  pe-parser-library.lib
  x86_64-softmmu.lib
  Dbghelp.lib
)

LINK_DIRECTORIES(${PROJECT_NAME}-test ${HARDWARE_EMU_LIB_DIRS})
add_executable(${PROJECT_NAME}-test ${HARDWARE_EMU_SOURCEFILES})
target_include_directories(${PROJECT_NAME}-test PUBLIC ${HARDWARE_EMU_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-test ${HARDWARE_EMU_LIBRARIES})

add_definitions(
    /std:c++17
    /DZYDIS_STATIC_DEFINE
)

set_target_properties(${PROJECT_NAME}-test PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")