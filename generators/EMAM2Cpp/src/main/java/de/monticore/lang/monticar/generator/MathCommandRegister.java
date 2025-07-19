/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.generator.cpp.commands.ArgumentNoReturnMathCommand;

import java.util.ArrayList;
import java.util.List;

/**
 */
public abstract class MathCommandRegister {
    public List<MathCommand> mathCommands = new ArrayList<>();
    public List<ArgumentNoReturnMathCommand> argumentNoReturnMathCommands = new ArrayList<>();

    public MathCommandRegister() {
        init();
    }

    public void registerMathCommand(MathCommand mathCommand) {
        mathCommands.add(mathCommand);
        if(mathCommand.isArgumentNoReturnMathCommand()){
            argumentNoReturnMathCommands.add((ArgumentNoReturnMathCommand) mathCommand);
        }
    }

    public MathCommand getMathCommand(String functionName) {
        for (MathCommand mathCommand : mathCommands) {
           if (mathCommand.getMathCommandName().equals(functionName))
                   return mathCommand;
        }
        return null;
    }

    public boolean isMathCommand(String functionName) {
        boolean isMathCommand = false;
        if (getMathCommand(functionName) != null) {
            isMathCommand = true;
        } else {
            isMathCommand = isTargetLanguageCommand(functionName);
        }
        return isMathCommand;
    }

    private boolean isTargetLanguageCommand(String command) {
        for (MathCommand mathCommand : mathCommands)
            if (mathCommand.isTargetLanguageCommand(command))
                return true;
        return false;
    }

    protected abstract void init();

}
