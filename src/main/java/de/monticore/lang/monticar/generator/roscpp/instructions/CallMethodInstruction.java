package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;

public class CallMethodInstruction extends TargetCodeInstruction {

    public CallMethodInstruction(String publishMethodName) {
        this.instruction = publishMethodName + "();";
    }
}
