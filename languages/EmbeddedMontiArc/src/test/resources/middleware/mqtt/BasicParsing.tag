/* (c) https://github.com/MontiCore/monticore */
package middleware.mqtt;
conforms to middleware.mqtt.MqttToEmamTagSchema;

tags Echo {
tag basicParsing.mqttInQ with MqttConnection = {topic = /echoQ};
tag basicParsing.mqttOutQ with MqttConnection = {topic = /echoQ};
tag basicParsing.mqttInN with MqttConnection = {topic = /echoN};
tag basicParsing.mqttOutN with MqttConnection = {topic = /echoN};
tag basicParsing.mqttInZ with MqttConnection = {topic = /echoZ};
tag basicParsing.mqttOutZ with MqttConnection = {topic = /echoZ};
tag basicParsing.mqttInB with MqttConnection = {topic = /echoB};
tag basicParsing.mqttOutB with MqttConnection = {topic = /echoB};
tag basicParsing.emptyTagIn with MqttConnection;
}
