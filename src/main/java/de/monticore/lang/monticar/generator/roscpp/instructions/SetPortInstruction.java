package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.Instruction;

public class SetPortInstruction implements Instruction {
    private String targetLanguageInstruction;

    public SetPortInstruction(PortSymbol portSymbol, String msgField) {
        targetLanguageInstruction = "component." + portSymbol.getName() + " = msg." + msgField + ";";
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
