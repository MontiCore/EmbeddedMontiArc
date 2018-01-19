package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.roscpp.RosTopic;

public class PublishInstruction implements Instruction {
    private String targetLanguageInstruction;

    public PublishInstruction(RosTopic rosTopic) {
        this.targetLanguageInstruction = rosTopic.getPublisher().orElse(null).getNameTargetLanguageFormat() + ".publish(tmpMsg);";
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
