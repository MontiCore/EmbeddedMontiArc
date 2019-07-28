<#--@formatter:off-->
# Setting cmake version
cmake_minimum_required(VERSION 3.1...3.14)
if(<#noparse>${CMAKE_VERSION}</#noparse> VERSION_LESS 3.12)
    cmake_policy(VERSION <#noparse>${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}</#noparse>)
endif()

# Using C++ version 11 with threads for compiling
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "<#noparse>${CMAKE_CXX_FLAGS}</#noparse> -std=c++11 -pthread")

# Setting project name and description
project(${model.getCompName()})

set(SEARCH_MQTT FALSE)

# Check is environment variable was set
IF(DEFINED ENV{MQTT_INCLUDE_DIR} AND DEFINED ENV{MQTT_LIBS})
  message("...MQTT environment variables are set" )
ELSE()
  message("MQTT environment variables are not defined.")
  # Find mqtt packages using modules
  find_package(MQTT REQUIRED)
  SET(SEARCH_MQTT TRUE)
ENDIF()

# Adding library as a target
add_library(MqttAdapter_${model.getEscapedCompName()}
Callback.cpp
MqttAdapter_${model.getEscapedCompName()}.cpp
${model.getEscapedCompName()}.cpp
Callback.hpp
MqttAdapter_${model.getEscapedCompName()}.h
${model.getEscapedCompName()}.h
)

list(APPEND LIBRARIES ${model.getCompName()})
list(APPEND LIBRARIES IAdapter_${model.getCompName()})

# Checking if everything was found
if(SEARCH_MQTT)
  if (NOT MQTT_C_LIB OR NOT MQTT_A_LIB OR NOT MQTT_PP_LIB)
    message(FATAL_ERROR "MQTT libraries not found!")
  elseif (NOT MQTT_INCLUDE_DIR)
    message(FATAL_ERROR "MQTT includes not found!")
  else()
    message("MQTT includes found in " <#noparse>${MQTT_INCLUDE_DIR}</#noparse>)
    message("MQTT libraries found in " <#noparse>${MQTT_LIBS}</#noparse>)
  endif()
endif()

# Adding include directory to a target
target_include_directories(MqttAdapter_${model.getEscapedCompName()} PUBLIC <#noparse>${MQTT_INCLUDE_DIR}</#noparse>)

# Linking libraries to target
target_link_libraries(MqttAdapter_${model.getEscapedCompName()} PUBLIC <#noparse>${MQTT_LIBS}</#noparse>)

# Export target to a cmake module file for outside usage
export(TARGETS MqttAdapter_${model.getEscapedCompName()} FILE MQTTAdapter_${model.getEscapedCompName()}.cmake)
