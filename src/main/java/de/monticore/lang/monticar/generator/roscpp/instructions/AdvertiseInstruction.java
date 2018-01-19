package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.roscpp.RosTopic;

public class AdvertiseInstruction implements Instruction {
    private static final int MSG_QUEUE_SIZE = 5;
    private String targetLanguageInstruction;

    public AdvertiseInstruction(Variable publisher, RosTopic rosTopic) {
        this.targetLanguageInstruction = publisher.getNameTargetLanguageFormat() + " = node_handle.advertise<" + rosTopic.getFullRosType() + ">(\"" + rosTopic.getName() + "\"," + MSG_QUEUE_SIZE + ");";
    }

    @Override
    public String getTargetLanguageInstruction() {
        return targetLanguageInstruction;
    }

    @Override
    public boolean isConnectInstruction() {
        return false;
    }

    @Override
    public boolean isTargetCodeInstruction() {
        return true;
    }

    @Override
    public boolean isExecuteInstruction() {
        return false;
    }
}
