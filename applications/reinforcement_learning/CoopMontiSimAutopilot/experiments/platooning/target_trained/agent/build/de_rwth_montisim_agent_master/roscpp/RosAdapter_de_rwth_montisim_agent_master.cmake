# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.6)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6...3.20)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget RosAdapter_de_rwth_montisim_agent_master)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Create imported target RosAdapter_de_rwth_montisim_agent_master
add_library(RosAdapter_de_rwth_montisim_agent_master STATIC IMPORTED)

set_target_properties(RosAdapter_de_rwth_montisim_agent_master PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/tilmohr/dev/coopmontisimautopilot/experiments/platooning/target/agent/src/de_rwth_montisim_agent_master/roscpp;/opt/ros/noetic/include;/usr/include;/opt/ros/noetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp;/opt/ros/noetic/include;/usr/include"
  INTERFACE_LINK_LIBRARIES "de_rwth_montisim_agent_master;IAdapter_de_rwth_montisim_agent_master;/opt/ros/noetic/lib/libroscpp.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0;/opt/ros/noetic/lib/librosconsole.so;/opt/ros/noetic/lib/librosconsole_log4cxx.so;/opt/ros/noetic/lib/librosconsole_backend_interface.so;/usr/lib/x86_64-linux-gnu/liblog4cxx.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0;/opt/ros/noetic/lib/libroscpp_serialization.so;/opt/ros/noetic/lib/libxmlrpcpp.so;/opt/ros/noetic/lib/librostime.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0;/opt/ros/noetic/lib/libcpp_common.so;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0;/usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4;/opt/ros/noetic/lib/libroscpp_serialization.so;/opt/ros/noetic/lib/librostime.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0;/opt/ros/noetic/lib/libcpp_common.so;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0;/usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4"
)

# Import target "RosAdapter_de_rwth_montisim_agent_master" for configuration ""
set_property(TARGET RosAdapter_de_rwth_montisim_agent_master APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(RosAdapter_de_rwth_montisim_agent_master PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "/home/tilmohr/dev/coopmontisimautopilot/experiments/platooning/target/agent/build/de_rwth_montisim_agent_master/roscpp/libRosAdapter_de_rwth_montisim_agent_master.a"
  )

# Make sure the targets which have been exported in some other
# export set exist.
unset(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets)
foreach(_target "de_rwth_montisim_agent_master" "IAdapter_de_rwth_montisim_agent_master" )
  if(NOT TARGET "${_target}" )
    set(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets "${${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets} ${_target}")
  endif()
endforeach()

if(DEFINED ${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets)
  if(CMAKE_FIND_PACKAGE_NAME)
    set( ${CMAKE_FIND_PACKAGE_NAME}_FOUND FALSE)
    set( ${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE "The following imported targets are referenced, but are missing: ${${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets}")
  else()
    message(FATAL_ERROR "The following imported targets are referenced, but are missing: ${${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets}")
  endif()
endif()
unset(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets)

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
