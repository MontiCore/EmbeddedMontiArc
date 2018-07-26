package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.roscpp.util.Method;
import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;

public class CallMethodInstruction extends TargetCodeInstruction {

    public CallMethodInstruction(Method publishMethod) {
        this.instruction = publishMethod.getName() + "();";
    }
}
