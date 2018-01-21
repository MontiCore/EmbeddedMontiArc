package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;

public class ConnectMsgToPortInstruction extends TargetCodeInstruction {

    public ConnectMsgToPortInstruction(PortSymbol portSymbol, Method convertMethod) {
        this.instruction = "component." + portSymbol.getName() + " = MsgPortHelper::" + convertMethod.getName() + "(msg);";
    }
}
