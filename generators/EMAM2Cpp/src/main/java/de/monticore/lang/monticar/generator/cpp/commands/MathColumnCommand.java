/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.EMAMBluePrint;
import de.monticore.lang.monticar.generator.MathCommand;

/**
 */
public class MathColumnCommand extends MathCommand {
    public MathColumnCommand() {
        setMathCommandName("col");
    }
    @Override
    public void convert(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrint bluePrint) {
        //Do nothing
    }
}
