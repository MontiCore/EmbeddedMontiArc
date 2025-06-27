/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

public class UpdateInstruction extends ExecuteInstruction {

    public UpdateInstruction(String componentName, EMAMBluePrint bluePrint, boolean canBeThreaded) {
        super(componentName, bluePrint, canBeThreaded);
        EXECUTE_COMMAND = "update";
    }

    public UpdateInstruction(ExecuteInstruction executeInstruction) {
        this(executeInstruction.componentName, executeInstruction.bluePrint, executeInstruction.canBeThreaded);
    }
}
