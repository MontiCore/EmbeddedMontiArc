/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.monticar.generator.MathCommand;

/**
 * @author Ahmed Diab.
 */
public abstract class ArgumentReturnMathCommand extends MathCommand{
    public ArgumentReturnMathCommand() {

    }

    public ArgumentReturnMathCommand(String argumentReturnMathCommandName) {
        super(argumentReturnMathCommandName);
    }

    @Override
    public boolean isArgumentReturnMathCommand() {
        return true;
    }
}
