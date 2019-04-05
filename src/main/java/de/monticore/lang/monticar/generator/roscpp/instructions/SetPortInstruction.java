package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.monticar.generator.roscpp.DirectMsgConverter;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;

public class SetPortInstruction extends TargetCodeInstruction {

    public SetPortInstruction(EMAPortSymbol portSymbol, DirectMsgConverter msgConverter) {
        if (portSymbol == null || msgConverter == null)
            throw new IllegalArgumentException();
        this.instruction = "component->" + NameHelper.getPortNameTargetLanguage(portSymbol) + " = " + msgConverter.getConversion(portSymbol) + ";";
    }
}
