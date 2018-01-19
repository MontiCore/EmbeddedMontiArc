package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.roscpp.MethodHelper;

public class SetMsgFieldInstruction implements Instruction {
    String targetLanguageInstruction;

    public SetMsgFieldInstruction(PortSymbol portSymbol, String msgField) {
        targetLanguageInstruction = "tmpMsg." + msgField + " = " + MethodHelper.getPortNameTargetLanguage(portSymbol) + ";";
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
