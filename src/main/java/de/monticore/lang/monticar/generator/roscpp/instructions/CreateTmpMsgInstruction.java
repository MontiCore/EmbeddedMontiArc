package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;

public class CreateTmpMsgInstruction extends TargetCodeInstruction {

    public CreateTmpMsgInstruction(String fullRosType) {
        this.instruction = fullRosType + " tmpMsg;";
    }
}
