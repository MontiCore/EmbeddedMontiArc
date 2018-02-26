package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;

import java.util.stream.Collectors;

public class SetStructMsgInstruction extends TargetCodeInstruction {
    public SetStructMsgInstruction(PortSymbol portSymbol, RosMsg rosMsg) {
        if (rosMsg.getName().startsWith("std_msgs/")) {
            this.instruction = NameHelper.getAllFieldNames(rosMsg).stream()
                    .map(field -> "tmpMsg->" + field + " = component->" + portSymbol.getName() + ";")
                    .sorted()
                    .collect(Collectors.joining("\n"));
        } else {
            this.instruction = NameHelper.getAllFieldNames(rosMsg).stream()
                    .map(field -> "tmpMsg->" + field + " = component->" + portSymbol.getName() + "." + field + ";")
                    .sorted()
                    .collect(Collectors.joining("\n"));
        }
    }
}
