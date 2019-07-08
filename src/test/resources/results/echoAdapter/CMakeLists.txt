# Setting cmake version
cmake_minimum_required(VERSION 3.1...3.14)
if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

# Using C++ version 11 with threads for compiling
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# Setting project name and description
project(tests.a.compA)

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
add_library(MqttAdapter_tests_a_compA
Callback.cpp
MqttAdapter_tests_a_compA.cpp
tests_a_compA.cpp
Callback.hpp
MqttAdapter_tests_a_compA.h
tests_a_compA.h
)

# Checking if everything was found
if(SEARCH_MQTT)
  if (NOT MQTT_C_LIB OR NOT MQTT_A_LIB OR NOT MQTT_PP_LIB)
    message(FATAL_ERROR "MQTT libraries not found!")
  elseif (NOT MQTT_INCLUDE_DIR)
    message(FATAL_ERROR "MQTT includes not found!")
  else()
    message("MQTT includes found in " ${MQTT_INCLUDE_DIR})
    message("MQTT libraries found in " ${MQTT_LIBS})
  endif()
endif()

# Adding include directory to a target
target_include_directories(MqttAdapter_tests_a_compA PUBLIC ${MQTT_INCLUDE_DIR})

# Linking libraries to target
target_link_libraries(MqttAdapter_tests_a_compA PUBLIC ${MQTT_LIBS})

# Export target to a cmake module file for outside usage
export(TARGETS MqttAdapter_tests_a_compA FILE MQTTAdapter_tests_a_compA.cmake)
