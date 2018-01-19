package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Method;

public class ConnectMsgToPortInstruction implements Instruction {
    private String targetLanguageInstruction;

    public ConnectMsgToPortInstruction(PortSymbol portSymbol, Method convertMethod) {
        this.targetLanguageInstruction = "component." + portSymbol.getName() + " = MsgPortHelper::" + convertMethod.getName() + "(msg);";
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
