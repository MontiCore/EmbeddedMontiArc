package de.monticore.lang.monticar.generator.roscpp.tagging;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

import java.util.Optional;

public class RosConnectionSymbol extends TagSymbol {
    public static final RosConnectionKind KIND = RosConnectionKind.INSTANCE;

    public RosConnectionSymbol(String topicName, String topicType) {
        this(KIND, topicName, topicType);
    }

    public RosConnectionSymbol(RosConnectionKind kind, String topicName, String topicType) {
        super(kind, topicName, topicType, Optional.empty());
    }

    public RosConnectionSymbol(String topicName, String topicType, String msgField) {
        this(KIND, topicName, topicType, msgField);
    }

    protected RosConnectionSymbol(RosConnectionKind kind, String topicName, String topicType, String msgField) {
        super(kind, topicName, topicType, Optional.of(msgField));
    }

    @Override
    public String toString() {
        return String.format("RosConnection = (%s, %s), %s",
                getTopicName(), getTopicType(), getMsgField());
    }

    public String getTopicName() {
        return getValue(0);
    }

    public String getTopicType() {
        return getValue(1);
    }

    public Optional<String> getMsgField() {
        return getValue(2);
    }

    public static class RosConnectionKind extends TagKind {
        public static final RosConnectionKind INSTANCE = new RosConnectionKind();

        protected RosConnectionKind() {
        }
    }
}