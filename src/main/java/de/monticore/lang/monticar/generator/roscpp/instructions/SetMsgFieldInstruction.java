package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.MethodHelper;

public class SetMsgFieldInstruction extends TargetCodeInstruction {

    public SetMsgFieldInstruction(PortSymbol portSymbol, String msgField) {
        this.instruction = "tmpMsg." + msgField + " = " + MethodHelper.getPortNameTargetLanguage(portSymbol) + ";";
    }
}
