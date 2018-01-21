package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.TargetCodeInstruction;

public class ExecuteComponentInstruction extends TargetCodeInstruction {

    public ExecuteComponentInstruction() {
        this.instruction = "component.execute();";
    }
}
