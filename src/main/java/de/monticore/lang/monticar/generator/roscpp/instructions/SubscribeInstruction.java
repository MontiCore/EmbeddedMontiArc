package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;

import java.util.Objects;

public class SubscribeInstruction extends TargetCodeInstruction {
    private static final int MSG_QUEUE_SIZE = 5;

    public SubscribeInstruction(String className, String nameInTargetFormat, String topicName, String callback, boolean isRos2, String fullRosType) {
        if (!isRos2) {
            this.instruction = nameInTargetFormat + " = node_handle.subscribe(\"" + topicName + "\" ," + MSG_QUEUE_SIZE + ",&" + className + "::" + callback + ", this, ros::TransportHints().tcpNoDelay());";
        } else {
            this.instruction = nameInTargetFormat + " = node_handle->create_subscription<" + fullRosType + ">(\"" + topicName + "\", std::bind(&" + className + "::" + callback + ", this, std::placeholders::_1));";
        }
    }
    @Override
    public boolean equals(Object other) {
        if (!(other instanceof SubscribeInstruction)) return false;

        return getTargetLanguageInstruction().equals(((SubscribeInstruction) other).getTargetLanguageInstruction());
    }

    @Override
    public int hashCode() {
        return Objects.hash(instruction);
    }

}
