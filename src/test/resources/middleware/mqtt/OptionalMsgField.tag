package middleware.mqtt;
conforms to middleware.mqtt.MqttToEmamTagSchema;

tags OptionalMsgField{
tag optionalMsgField.in1 with MqttConnection = {topic = name1};
tag optionalMsgField.out1 with MqttConnection = {topic = name1, msgField = msgField1};
}