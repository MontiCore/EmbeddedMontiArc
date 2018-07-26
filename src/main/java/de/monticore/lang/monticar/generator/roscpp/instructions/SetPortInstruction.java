package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.MsgConverter;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;

public class SetPortInstruction extends TargetCodeInstruction {

    public SetPortInstruction(EMAPortSymbol portSymbol, MsgConverter msgConverter) {
        if (portSymbol == null || msgConverter == null)
            throw new IllegalArgumentException();
        this.instruction = "component->" + NameHelper.getPortNameTargetLanguage(portSymbol) + " = " + msgConverter.getConversion(portSymbol) + ";";
    }
}
