# Created by Georg Vinogradov on 28.05.19

# Setting cmake version
cmake_minimum_required(VERSION 3.1...3.14)
if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

# Using C++ version 11 with threads for compiling
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# Setting search paths for cmake modules
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")

# Setting project name and description
project(MQTT VERSION 1.0
                  DESCRIPTION "MQTT demo"
                  LANGUAGES CXX)

# Find mqtt packages using modules
find_package(MQTT REQUIRED)

# Find Pthread package
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Adding executable target
add_executable(mqtt_demo main.cpp)

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
