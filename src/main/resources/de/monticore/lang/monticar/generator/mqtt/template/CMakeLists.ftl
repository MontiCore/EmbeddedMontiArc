#Setting cmake version
cmake_minimum_required(VERSION 3.1...3.14)
if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

#Using C++ version 11 with threads for compiling
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# Find Pthread package
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#Setting Project name
project (${model.name})

#Find MQTT packages
message("Searching for installed MQTT on your system...")

# Searching for include directory
if (NOT MQTT_INCLUDE_DIR)
  find_path(MQTT_INCLUDE_DIR MQTTAsync.h HINTS "/usr/include/" "/usr/local/include/")
endif()

# Searching for libraries (without SSL support)
if (NOT MQTT_LIBS)
  find_library(
    MQTT_C_LIB
  NAMES paho-mqtt3c)
  find_library(
    MQTT_A_LIB
  NAMES paho-mqtt3a)
  find_library(
    MQTT_PP_LIB
  NAMES paho-mqttpp3)
  set(MQTT_LIBS ${MQTT_A_LIB} ${MQTT_C_LIB} ${MQTT_PP_LIB})
endif()

add_library(${model.name} ${model.name}.cpp)

# Checking if everything was found
if (NOT MQTT_C_LIB OR NOT MQTT_A_LIB OR NOT MQTT_PP_LIB)
  message(FATAL_ERROR "MQTT libraries not found!")
elseif (NOT MQTT_INCLUDE_DIR)
  message(FATAL_ERROR "MQTT includes not found!")
else()
  message("MQTT includes found in "${MQTT_INCLUDE_DIR})
  message("MQTT libraries found in "${MQTT_LIBS})
endif()

# Adding include directory to a target
target_include_directories(mqtt_demo PUBLIC ${MQTT_INCLUDE_DIR})

# Linking libraries to target
target_link_libraries(mqtt_demo PUBLIC ${MQTT_LIBS})

list(APPEND LIBRARIES ${model.compName})
list(APPEND LIBRARIES IAdapter_${model.compName})
<#list model.getPackages() as lib>
list(APPEND LIBRARIES ${lib}_LIBRARIES})
</#list>
target_link_libraries(${model.name} <#noparse>${LIBRARIES}</#noparse>)

list(APPEND INCLUDE_DIRS <#noparse>${CMAKE_CURRENT_SOURCE_DIR}</#noparse>)
<#list model.getPackages() as incl>
list(APPEND INCLUDE_DIRS ${incl}_INCLUDE_DIRS})
</#list>
target_include_directories(${model.name} PUBLIC <#noparse>${INCLUDE_DIRS}</#noparse>)

export(TARGETS ${model.name} FILE ${model.name}.cmake)
