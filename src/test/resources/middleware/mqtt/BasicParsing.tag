package middleware.mqtt;
conforms to middleware.mqtt.MqttToEmamTagSchema;

tags Echo {
tag basicParsing.mqttIn with MqttConnection = {topic = /clock, msgField = clock.toSec()};
tag basicParsing.mqttOut with MqttConnection = {topic = /echo, msgField = data};
tag basicParsing.emptyTagIn with MqttConnection;
}