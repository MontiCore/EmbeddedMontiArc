package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;

public class CallPublishInstruction extends TargetCodeInstruction {

    public CallPublishInstruction(Method publishMethod) {
        this.instruction = publishMethod.getName() + "();";
    }
}
