cmake_minimum_required(VERSION 3.7)
project(hardware-emulator)

#Required Packages
find_package(Threads)

#Find java binary, extract directory to use JNI.
# find_package(Java COMPONENTS Development)
# get_filename_component(JAVA_BIN_DIR "${Java_JAVAC_EXECUTABLE}" PATH)
# set(JAVA_DIR "${JAVA_BIN_DIR}/..")
# message("JAVA_DIR: ${JAVA_DIR}")

find_package(JNI)
if (JNI_FOUND)
#message("JAVA_INCLUDE_PATH: ${JAVA_INCLUDE_PATH}")
#message("JAVA_INCLUDE_PATH2: ${JAVA_INCLUDE_PATH2}")
#message("JAVA_AWT_LIBRARY: ${JAVA_AWT_LIBRARY}")
#message("JAVA_JVM_LIBRARY: ${JAVA_JVM_LIBRARY}")
endif()

if (NOT JNI_FOUND)
message("Could not find JNI")
endif()

#Name of the resulting library that is loaded in the autopilot RMIServer.
set(AUTOPILO_ADAPTER_NAME "HardwareEmulator")

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
)

#Source files for the test project
list(APPEND HARDWARE_EMU_TEST_SOURCEFILES
    src/main.cpp
)

#Source files for the Autopilot Library
list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_SOURCEFILES
  src/emulator/emulator_server.h
  src/emulator/emulator_server.cpp
)

#Setup the project file structure for Visual Studio projects
foreach(source IN LISTS HARDWARE_EMU_SOURCEFILES HARDWARE_EMU_TEST_SOURCEFILES HARDWARE_EMU_AUTOPILOT_ADAPTER_SOURCEFILES)
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
)

#Library dependencies
list(APPEND HARDWARE_EMU_LIBRARIES
    unicorn
    Zydis
    pe-parser-library
)

#Linux include/lib dirs and library dependencies
if ( CMAKE_COMPILER_IS_GNUCC )
  list(APPEND HARDWARE_EMU_INCLUDE_DIRS
    ../zydis/build
    ../zydis/dependencies/zycore/build
    "${JAVA_INCLUDE_PATH}"
    "${JAVA_INCLUDE_PATH2}"
  )
  list(APPEND HARDWARE_EMU_LIB_DIRS
    ../zydis/build
    "../pe-parse/${CMAKE_BUILD_TYPE}/pe-parser-library"
    ../unicorn
    ../unicorn/qemu
  )
  message("Added lib dir: \"../pe-parse/${CMAKE_BUILD_TYPE}/pe-parser-library\"")
  list(APPEND HARDWARE_EMU_LIBRARIES
    pthread
    stdc++fs
  )
endif()

#MSVC include/lib dirs and library dependencies
if ( MSVC )
  list(APPEND HARDWARE_EMU_INCLUDE_DIRS
    ../zydis/msvc
    "${JAVA_INCLUDE_PATH}"
    "${JAVA_INCLUDE_PATH2}"
  )
  #list(APPEND HARDWARE_EMU_LIB_DIRS
  #  ../zydis/build
  #  ../pe-parse/build/pe-parser-library
  #  ../unicorn/build
  #  ../unicorn/build/qemu
  #)
  list(APPEND HARDWARE_EMU_LIB_DIRS
    libs
  )
  list(APPEND HARDWARE_EMU_LIBRARIES
    Dbghelp
    x86_64-softmmu
  )
endif()


list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_LIBRARIES
  "${JAVA_AWT_LIBRARY}"
  "${JAVA_JVM_LIBRARY}"
)



#Test project
LINK_DIRECTORIES(${PROJECT_NAME}-test ${HARDWARE_EMU_LIB_DIRS})
add_executable(${PROJECT_NAME}-test ${HARDWARE_EMU_SOURCEFILES} ${HARDWARE_EMU_TEST_SOURCEFILES})
target_include_directories(${PROJECT_NAME}-test PUBLIC ${HARDWARE_EMU_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-test ${HARDWARE_EMU_LIBRARIES})

#Compiler options
if ( CMAKE_COMPILER_IS_GNUCC )
  add_definitions(
    #-std=c++17
    /DZYDIS_STATIC_DEFINE
  )
endif()
if ( MSVC )
  add_definitions(
    /std:c++17
    /DZYDIS_STATIC_DEFINE
    /D_CRT_SECURE_NO_WARNINGS
  )
endif()

#Visual Studio debugging directory
set_target_properties(${PROJECT_NAME}-test PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")


#AutopilotAdapter library
LINK_DIRECTORIES(${AUTOPILO_ADAPTER_NAME} ${HARDWARE_EMU_LIB_DIRS} ${HARDWARE_EMU_AUTOPILOT_ADAPTER_LIB_DIRS})
add_library(${AUTOPILO_ADAPTER_NAME} SHARED ${HARDWARE_EMU_SOURCEFILES} ${HARDWARE_EMU_AUTOPILOT_ADAPTER_SOURCEFILES})
target_include_directories(${AUTOPILO_ADAPTER_NAME} PUBLIC ${HARDWARE_EMU_INCLUDE_DIRS} ${HARDWARE_EMU_AUTOPILOT_ADAPTER_INCLUDE_DIRS} )
target_link_libraries(${AUTOPILO_ADAPTER_NAME} ${HARDWARE_EMU_LIBRARIES} ${HARDWARE_EMU_AUTOPILOT_ADAPTER_LIBRARIES})

#Compiler options
if ( CMAKE_COMPILER_IS_GNUCC )
    add_definitions(
      -std=c++17
      -fPIC
      /DZYDIS_STATIC_DEFINE
    )
endif()
if ( MSVC )
    add_definitions(
      /std:c++17
      /DZYDIS_STATIC_DEFINE
      /D_CRT_SECURE_NO_WARNINGS
    )
endif()

