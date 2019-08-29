package tests.mqtt;
conforms to de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttToEmamTagSchema;

tags Test {
tag testComp.in1 with MqttConnection = {topic = /clock, msgField = clock.toSec()};
}
