package de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt;

import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.tagging._symboltable.TagKind;

import java.util.Optional;

public class MqttConnectionSymbol extends MiddlewareSymbol {

    public static final MqttConnectionKind KIND = MqttConnectionKind.INSTANCE;

    public MqttConnectionSymbol() {
        super(KIND, Optional.empty(), Optional.empty(), Optional.empty(), Optional.empty());
    }

    public MqttConnectionSymbol(String id, String topicName) {
        this(KIND, id, topicName);
    }

    public MqttConnectionSymbol(MqttConnectionKind kind, String id, String topicName) {
        super(kind, Optional.ofNullable(id), Optional.ofNullable(topicName), Optional.empty());
    }

    public MqttConnectionSymbol(String id, String topicName, int qoS) {
        this(KIND, id, topicName, qoS);
    }

    public MqttConnectionSymbol(MqttConnectionKind kind, String id, String topicName, int qoS){
        super(kind, Optional.ofNullable(id), Optional.ofNullable(topicName), Optional.ofNullable(qoS));
    }

    public MqttConnectionSymbol(String id, String topicName, int qoS, String msgField){
        this(KIND, id, topicName, qoS, msgField);
    }

    protected MqttConnectionSymbol(MqttConnectionKind kind, String id, String topicName, int qoS, String msgField) {
        super(kind, Optional.ofNullable(id), Optional.ofNullable(topicName), Optional.ofNullable(qoS), Optional.ofNullable(msgField));
    }

    @Override
    public String toString() {
        return String.format("MqttConnection = %s, (%s, %d), %s",
                getId(), getTopicName(), getQoS(), getMsgField());
    }

    public Optional<String> getId() {
        return getValue(0);
    }

    public void setId(String id) {
        this.value.set(0, Optional.ofNullable(id));
    }

    public Optional<String> getTopicName() {
        return getValue(1);
    }

    public void setTopicName(String topicName) {
        this.values.set(1, Optional.ofNullable(topicName));
    }

    public Optional<int> getQoS() {
        return getValue(2);
    }

    public void setTopicType(String qoS) {
        this.values.set(2, Optional.ofNullable(qoS));
    }

    public Optional<String> getMsgField() {
        return getValue(3);
    }

    public void setMsgField(String msgField) {
        this.values.set(3, Optional.ofNullable(msgField));
    }

    public static class MqttConnectionKind extends TagKind {
        public static final MqttConnectionKind INSTANCE = new MqttConnectionKind();

        protected MqttConnectionKind() {
        }
    }
}
