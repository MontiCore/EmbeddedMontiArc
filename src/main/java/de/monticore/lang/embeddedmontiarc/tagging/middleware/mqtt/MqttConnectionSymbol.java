/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt;

import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.tagging._symboltable.TagKind;

import java.util.Optional;

public class MqttConnectionSymbol extends MiddlewareSymbol {

    public static final MqttConnectionKind KIND = MqttConnectionKind.INSTANCE;

    public MqttConnectionSymbol() {
        super(KIND, Optional.empty(), Optional.empty());
    }

    public MqttConnectionSymbol(String topicName) {
        this(KIND, topicName);
    }

    public MqttConnectionSymbol(MqttConnectionKind kind, String topicName) {
        super(kind, Optional.ofNullable(topicName), Optional.empty());
    }

    public MqttConnectionSymbol(String topicName, String msgField) {
        this(KIND, topicName, msgField);
    }

    protected MqttConnectionSymbol(MqttConnectionKind kind, String topicName, String msgField) {
        super(kind, Optional.ofNullable(topicName), Optional.ofNullable(msgField));
    }

    @Override
    public String toString() {
        return String.format("MqttConnection = %s, %s",
                getTopicName(), getMsgField());
    }

    public Optional<String> getTopicName() {
        return getValue(0);
    }

    public void setTopicName(String topicName) {
        this.values.set(0, Optional.ofNullable(topicName));
    }

    public Optional<String> getMsgField() {
        return getValue(1);
    }

    public void setMsgField(String msgField) {
        this.values.set(1, Optional.ofNullable(msgField));
    }

    public static class MqttConnectionKind extends TagKind {
        public static final MqttConnectionKind INSTANCE = new MqttConnectionKind();

        protected MqttConnectionKind() {
        }
    }
}
