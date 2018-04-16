package de.monticore.lang.monticar.generator;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public abstract class MathCommandRegister {
    public List<MathCommand> mathCommands = new ArrayList<>();

    public MathCommandRegister() {
        init();
    }

    public void registerMathCommand(MathCommand mathCommand) {
        mathCommands.add(mathCommand);
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
        if (!functionName.isEmpty()) {
            if (getMathCommand(functionName) != null) {
                isMathCommand = true;
            } else {
                isMathCommand = isTargetLanguageCommand(functionName);
            }
        }
        return isMathCommand;
    }

    private boolean isTargetLanguageCommand(String command) {
        for (MathCommand mathCommand : mathCommands)
            for (String s : mathCommand.getTargetLanguageCommandNames())
                if (s.contains(command))
                    return true;
        return false;
    }

    protected abstract void init();

}
