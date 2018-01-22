package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;

public class SetPortInstruction extends TargetCodeInstruction {

    public SetPortInstruction(PortSymbol portSymbol, String msgField) {
        if (portSymbol == null || msgField == null)
            throw new IllegalArgumentException();
        this.instruction = "component." + portSymbol.getName() + " = msg." + msgField + ";";
    }
}
