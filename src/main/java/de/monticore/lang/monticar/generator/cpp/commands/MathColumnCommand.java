package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.MathCommand;

/**
 * @author Sascha Schneiders
 */
public class MathColumnCommand extends MathCommand {
    public MathColumnCommand() {
        setMathCommandName("col");
    }
    @Override
    public void convert(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        //Do nothing
    }
}
