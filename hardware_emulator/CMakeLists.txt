cmake_minimum_required(VERSION 3.7)
project(hardware-emulator)

#Required Packages
find_package(Threads)

#Find java binary, extract directory to use JNI.
find_package(Java COMPONENTS Development)
get_filename_component(JAVA_BIN_DIR "${Java_JAVA_EXECUTABLE}" PATH)
set(JAVA_DIR "${JAVA_BIN_DIR}/..")
message("JAVA_DIR: ${JAVA_DIR}")

#Name of the resulting library that is loaded in the autopilot RMIServer.
set(AUTOPILO_ADAPTER_NAME "AutopilotAdapter")

#Basic source files
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

#Source files for the test project
list(APPEND HARDWARE_EMU_TEST_SOURCEFILES
    src/main.cpp
)

#Source files for the Autopilot Library
list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_SOURCEFILES
  include/hardware-emulator/jni/simulator_integration_AutopilotAdapter.h
  src/jni/AutopilotAdapter.cpp
)

#Setup the project file structure for Visual Studio projects
foreach(source IN LISTS HARDWARE_EMU_SOURCEFILES HARDWARE_EMU_TEST_SOURCEFILES HARDWARE_EMU_AUTOPILOT_ADAPTER_SOURCEFILES)
    string(REPLACE "include/hardware-emulator/" "" trunc_source "${source}")
    string(REPLACE "src/" "" trunc_source2 "${trunc_source}")
    get_filename_component(source_path "${trunc_source2}" PATH)
    string(REPLACE "/" "\\" source_path_msvc "${source_path}")
    source_group("${source_path_msvc}" FILES "${source}")
endforeach()


#Include directories
list(APPEND HARDWARE_EMU_INCLUDE_DIRS
  include/hardware-emulator
  ../unicorn/include
  ../pe-parse/pe-parser-library/include
  ../zydis/include
  ../zydis/dependencies/zycore/include
)

#Library directories
list(APPEND HARDWARE_EMU_LIB_DIRS
  ../zydis/build
  ../unicorn/build
  ../unicorn/build/qemu
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
    "${JAVA_DIR}/include"
    "${JAVA_DIR}/include/linux"
  )
  list(APPEND HARDWARE_EMU_LIB_DIRS
    "../pe-parse/${CMAKE_BUILD_TYPE}/pe-parser-library"
    ../unicorn
    ../unicorn/qemu
  )
  message("Added lib dir: \"../pe-parse/${CMAKE_BUILD_TYPE}/pe-parser-library\"")
  list(APPEND HARDWARE_EMU_LIBRARIES
    pthread
  )
endif()

#MSVC include/lib dirs and library dependencies
if ( MSVC )
  list(APPEND HARDWARE_EMU_INCLUDE_DIRS
    ../zydis/msvc
    "${JAVA_DIR}/include/win32"
    "${JAVA_DIR}/include"
  )
  list(APPEND HARDWARE_EMU_LIB_DIRS
    ../pe-parse/build/pe-parser-library
    ../unicorn/build
    ../unicorn/build/qemu
  )
  list(APPEND HARDWARE_EMU_LIBRARIES
    Dbghelp
    x86_64-softmmu
  )
endif()


#AutopilotAdapter specific include/lib dirs and library dependencies
if ( CMAKE_COMPILER_IS_GNUCC )
  list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_LIB_DIRS
    "${JAVA_DIR}/lib"
  )
  list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_LIBRARIES
    jawt
    java
  )
endif()
if ( MSVC )
  list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_LIB_DIRS
    "${JAVA_DIR}/lib"
  )
  list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_LIBRARIES
    jawt
    jvm
  )
endif()
list(APPEND HARDWARE_EMU_AUTOPILOT_ADAPTER_INCLUDE_DIRS
  
)


#Test project
LINK_DIRECTORIES(${PROJECT_NAME}-test ${HARDWARE_EMU_LIB_DIRS})
add_executable(${PROJECT_NAME}-test ${HARDWARE_EMU_SOURCEFILES} ${HARDWARE_EMU_TEST_SOURCEFILES})
target_include_directories(${PROJECT_NAME}-test PUBLIC ${HARDWARE_EMU_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-test ${HARDWARE_EMU_LIBRARIES})

#Compiler options
if ( CMAKE_COMPILER_IS_GNUCC )
  add_definitions(
    -std=c++17
    /DZYDIS_STATIC_DEFINE
  )
endif()
if ( MSVC )
  add_definitions(
    /std:c++17
    /DZYDIS_STATIC_DEFINE
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
    )
endif()

