package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.roscpp.RosTopic;

public class AdvertiseInstruction extends TargetCodeInstruction {
    private static final int MSG_QUEUE_SIZE = 5;

    public AdvertiseInstruction(RosTopic rosTopic) {
        Variable publisher = rosTopic.getPublisher()
                .orElseThrow(() -> new IllegalArgumentException("rosTopic.publisher must be set!"));
        this.instruction = publisher.getNameTargetLanguageFormat() + " = node_handle.advertise<" + rosTopic.getFullRosType() + ">(\"" + rosTopic.getName() + "\"," + MSG_QUEUE_SIZE + ");";
    }

    @Override
    public boolean equals(Object other) {
        if (!(other instanceof AdvertiseInstruction)) return false;

        return getTargetLanguageInstruction().equals(((AdvertiseInstruction) other).getTargetLanguageInstruction());
    }
}
