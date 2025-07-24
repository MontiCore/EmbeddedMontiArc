# (c) https://github.com/MontiCore/monticore  
# Created by Georg Vinogradov on 28.05.19

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
