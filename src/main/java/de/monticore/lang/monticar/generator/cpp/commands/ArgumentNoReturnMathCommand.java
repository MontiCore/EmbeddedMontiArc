/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.monticar.generator.MathCommand;

/**
 * @author Ahmed Diab.
 */
public abstract class ArgumentNoReturnMathCommand extends MathCommand{
    public ArgumentNoReturnMathCommand() {

    }

    public ArgumentNoReturnMathCommand(String argumentNoReturnMathCommandName) {
        super(argumentNoReturnMathCommandName);
    }

    @Override
    public boolean isArgumentNoReturnMathCommand() {
        return true;
    }
}
