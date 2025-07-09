/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

public class OutputInstruction extends ExecuteInstruction {

    public OutputInstruction(String componentName, EMAMBluePrint bluePrint, boolean canBeThreaded) {
        super(componentName, bluePrint, canBeThreaded);
        EXECUTE_COMMAND = "output";
    }

    public OutputInstruction(ExecuteInstruction executeInstruction) {
        this(executeInstruction.componentName, executeInstruction.bluePrint, executeInstruction.canBeThreaded);
    }
}
