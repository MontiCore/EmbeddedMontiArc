package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.util.Variable;

import java.util.Objects;

public class AdvertiseInstruction extends TargetCodeInstruction {
    private static final int MSG_QUEUE_SIZE = 5;

    public AdvertiseInstruction(Variable publisher, String fullRosType, String topicName, boolean isRos2) {
        if (!isRos2) {
            this.instruction = publisher.getNameTargetLanguageFormat() + " = node_handle.advertise<" + fullRosType + ">(\"" + topicName + "\"," + MSG_QUEUE_SIZE + ");";
        }else{
            this.instruction =  publisher.getNameTargetLanguageFormat() + " = node_handle->create_publisher<" + fullRosType + ">(\"" + topicName + "\");";
        }
    }
    @Override
    public boolean equals(Object other) {
        if (!(other instanceof AdvertiseInstruction)) return false;

        return getTargetLanguageInstruction().equals(((AdvertiseInstruction) other).getTargetLanguageInstruction());
    }

    @Override
    public int hashCode() {
        return Objects.hash(instruction);
    }
}
