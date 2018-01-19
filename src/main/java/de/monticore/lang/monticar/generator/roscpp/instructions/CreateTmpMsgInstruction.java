package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.roscpp.RosTopic;

public class CreateTmpMsgInstruction implements Instruction {
    String targetLanguageInstruction;

    public CreateTmpMsgInstruction(RosTopic rosTopic) {
        targetLanguageInstruction = rosTopic.getFullRosType() + " tmpMsg;";
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
