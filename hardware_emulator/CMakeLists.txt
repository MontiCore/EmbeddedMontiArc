cmake_minimum_required(VERSION 3.7)

project(hardware-emulator)

#Required Packages
find_package(Threads)
find_package(JNI REQUIRED)

set(LIBS_DIR "UNKNOWN")
if (CMAKE_HOST_WIN32)
  list(APPEND HARDWARE_EMU_LIBRARIES
  Dbghelp
  )

  if (MINGW)
    set(LIBS_DIR "libs/mingw")
  elseif(MSVC)
    set(LIBS_DIR "libs/vs")
  endif()

elseif(CMAKE_HOST_UNIX)
  set(LIBS_DIR "libs/unix")

  list(APPEND HARDWARE_EMU_LIBRARIES
    dl
  )
endif()

set(LIBRARY_NAME "HardwareEmulator")

#Basic source files
list(APPEND HARDWARE_EMU_SOURCEFILES
  src/debug.h
  src/debug.cpp
  src/tests.h
  src/tests.cpp
  src/utility.h
  src/utility.cpp
  src/config.h
  src/config.cpp
  
  src/dll_interface.h
  src/dll_interface.cpp
  
  
  src/emulator/emulator_manager.h
  src/emulator/emulator_manager.cpp
  src/emulator/hardware_emulator.h
  src/emulator/hardware_emulator.cpp
  src/emulator/function_value.h
  src/emulator/function_value.cpp
  
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

#Source files for the test project
list(APPEND HARDWARE_EMU_TEST_SOURCEFILES
    src/main.cpp
)

#Source files for the Autopilot Library
list(APPEND HARDWARE_EMU_LIB_SOURCEFILES
  src/emulator/de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_emulator_HardwareEmulatorInterface.h
  src/emulator/emulator_server.cpp
)

#Setup the project file structure for Visual Studio projects (OPTIONAL)
foreach(source IN LISTS HARDWARE_EMU_SOURCEFILES HARDWARE_EMU_TEST_SOURCEFILES HARDWARE_EMU_LIB_SOURCEFILES)
    string(REPLACE "src/" "" trunc_source "${source}")
    get_filename_component(source_path "${trunc_source}" PATH)
    string(REPLACE "/" "\\" source_path_msvc "${source_path}")
    source_group("${source_path_msvc}" FILES "${source}")
endforeach()


#Include directories
list(APPEND HARDWARE_EMU_INCLUDE_DIRS
  src
  ../unicorn/include
  ../pe-parse/pe-parser-library/include
  ../zydis/include
  ../zydis/dependencies/zycore/include
  "${JAVA_INCLUDE_PATH}"
  "${JAVA_INCLUDE_PATH2}"
)

#Library dependencies
list(APPEND HARDWARE_EMU_LIBRARIES
    unicorn
    Zydis
    pe-parser-library
)

list(APPEND HARDWARE_EMU_LIB_LIBRARIES
  "${JAVA_AWT_LIBRARY}"
  "${JAVA_JVM_LIBRARY}"
)

#Libraries directory
list(APPEND HARDWARE_EMU_LIB_DIRS
  "${LIBS_DIR}"
)

if ( CMAKE_COMPILER_IS_GNUCC )
  add_definitions(
    -std=c++17
    -fPIC
    -fpermissive
    -static-libstdc++
    /DZYDIS_STATIC_DEFINE
  )
  list(APPEND HARDWARE_EMU_INCLUDE_DIRS
    ../zydis/gnucc
  )
  list(APPEND HARDWARE_EMU_LIBRARIES
    pthread
    stdc++fs
  )
endif()

if ( MSVC )
  add_definitions(
    /std:c++17
    /DZYDIS_STATIC_DEFINE
    /D_CRT_SECURE_NO_WARNINGS
  )
  list(APPEND HARDWARE_EMU_INCLUDE_DIRS
    ../zydis/msvc
  )
  list(APPEND HARDWARE_EMU_LIBRARIES
    Dbghelp
    x86_64-softmmu
  )
endif()






#Test project
LINK_DIRECTORIES(${PROJECT_NAME}-test ${HARDWARE_EMU_LIB_DIRS})
add_executable(${PROJECT_NAME}-test ${HARDWARE_EMU_SOURCEFILES} ${HARDWARE_EMU_TEST_SOURCEFILES})
target_include_directories(${PROJECT_NAME}-test PUBLIC ${HARDWARE_EMU_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-test ${HARDWARE_EMU_LIBRARIES})

#Visual Studio debugging directory
set_target_properties(${PROJECT_NAME}-test PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")


#HardwareEmulatorLibrary
LINK_DIRECTORIES(${LIBRARY_NAME} ${HARDWARE_EMU_LIB_DIRS} ${HARDWARE_EMU_LIB_LIB_DIRS})
add_library(${LIBRARY_NAME} SHARED ${HARDWARE_EMU_SOURCEFILES} ${HARDWARE_EMU_LIB_SOURCEFILES})
target_include_directories(${LIBRARY_NAME} PUBLIC ${HARDWARE_EMU_INCLUDE_DIRS} ${HARDWARE_EMU_LIB_INCLUDE_DIRS} )
target_link_libraries(${LIBRARY_NAME} ${HARDWARE_EMU_LIBRARIES} ${HARDWARE_EMU_LIB_LIBRARIES})

set(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/install")
set_target_properties(${LIBRARY_NAME} PROPERTIES PREFIX "")
string(TOLOWER "${CMAKE_SYSTEM_NAME}" SYS_NAME)
install(
  TARGETS ${LIBRARY_NAME} 
  RUNTIME
    DESTINATION "${PROJECT_SOURCE_DIR}/../src/main/resources/${SYS_NAME}"
    LIBRARY DESTINATION "${PROJECT_SOURCE_DIR}/../src/main/resources/${SYS_NAME}"
    CONFIGURATIONS Release
)

