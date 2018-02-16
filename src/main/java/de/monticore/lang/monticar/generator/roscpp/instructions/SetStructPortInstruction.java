package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;

import java.util.stream.Collectors;

public class SetStructPortInstruction extends TargetCodeInstruction {
    public SetStructPortInstruction(PortSymbol port, RosMsg rosMsg) {
        this.instruction = NameHelper.getAllFieldNames(rosMsg).stream()
                .map(field -> "component->" + NameHelper.getPortNameTargetLanguage(port) + "." + field + " = msg->" + field + ";")
                .sorted()
                .collect(Collectors.joining("\n"));
    }

}
