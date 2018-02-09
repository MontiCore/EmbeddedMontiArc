package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.MsgConverter;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;

public class SetPortInstruction extends TargetCodeInstruction {

    public SetPortInstruction(PortSymbol portSymbol, MsgConverter msgConverter) {
        if (portSymbol == null || msgConverter == null)
            throw new IllegalArgumentException();
        this.instruction = "component." + NameHelper.getPortNameTargetLanguage(portSymbol) + " = " + msgConverter.getConversion(portSymbol) + ";";
    }
}
