package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.roscpp.RosTopic;

public class AdvertiseInstruction extends TargetCodeInstruction {
    private static final int MSG_QUEUE_SIZE = 5;

    public AdvertiseInstruction(Variable publisher, RosTopic rosTopic) {
        this.instruction = publisher.getNameTargetLanguageFormat() + " = node_handle.advertise<" + rosTopic.getFullRosType() + ">(\"" + rosTopic.getName() + "\"," + MSG_QUEUE_SIZE + ");";
    }
}
