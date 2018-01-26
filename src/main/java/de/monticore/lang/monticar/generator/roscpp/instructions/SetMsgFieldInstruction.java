package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.MsgConverter;

public class SetMsgFieldInstruction extends TargetCodeInstruction {

    public SetMsgFieldInstruction(PortSymbol portSymbol, MsgConverter msgConverter) {
        this.instruction = "tmpMsg" + msgConverter.getConversion(portSymbol) + ";";
    }
}
