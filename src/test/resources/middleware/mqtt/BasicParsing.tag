package middleware.mqtt;
conforms to middleware.mqtt.MqttToEmamTagSchema;

tags Echo {
tag basicParsing.mqttInQ with MqttConnection = {topic = /echoQ, msgField = dataQ};
tag basicParsing.mqttOutQ with MqttConnection = {topic = /echoQ};
tag basicParsing.mqttInN with MqttConnection = {topic = /echoN, msgField = dataN};
tag basicParsing.mqttOutN with MqttConnection = {topic = /echoN};
tag basicParsing.mqttInZ with MqttConnection = {topic = /echoZ, msgField = dataZ};
tag basicParsing.mqttOutZ with MqttConnection = {topic = /echoZ};
tag basicParsing.mqttInB with MqttConnection = {topic = /echoB, msgField = true};
tag basicParsing.mqttOutB with MqttConnection = {topic = /echoB};
tag basicParsing.emptyTagIn with MqttConnection;
}
