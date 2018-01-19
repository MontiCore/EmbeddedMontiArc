package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Method;

public class CallPublishInstruction implements Instruction {
    String targetLanguageInstruction;

    public CallPublishInstruction(Method publishMethod) {
        targetLanguageInstruction = publishMethod.getName() + "();";
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
