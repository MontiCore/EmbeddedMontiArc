package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.RosTopic;

public class CreateTmpMsgInstruction extends TargetCodeInstruction {

    public CreateTmpMsgInstruction(RosTopic rosTopic) {
        this.instruction = rosTopic.getFullRosType() + " tmpMsg;";
    }
}
