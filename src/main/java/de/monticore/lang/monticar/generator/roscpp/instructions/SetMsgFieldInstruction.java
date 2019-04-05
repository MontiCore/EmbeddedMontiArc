package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.monticar.generator.roscpp.DirectMsgConverter;
import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;

public class SetMsgFieldInstruction extends TargetCodeInstruction {

    public SetMsgFieldInstruction(EMAPortSymbol portSymbol, DirectMsgConverter msgConverter) {
        this.instruction = "tmpMsg" + msgConverter.getConversion(portSymbol) + ";";
    }
}
