package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.MsgConverter;

public class SetMsgFieldInstruction extends TargetCodeInstruction {

    public SetMsgFieldInstruction(EMAPortSymbol portSymbol, MsgConverter msgConverter) {
        this.instruction = "tmpMsg" + msgConverter.getConversion(portSymbol) + ";";
    }
}
