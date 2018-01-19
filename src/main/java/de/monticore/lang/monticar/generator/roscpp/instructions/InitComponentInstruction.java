package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.Instruction;

public class InitComponentInstruction implements Instruction {
    @Override
    public String getTargetLanguageInstruction() {
        return "component.init();";
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
