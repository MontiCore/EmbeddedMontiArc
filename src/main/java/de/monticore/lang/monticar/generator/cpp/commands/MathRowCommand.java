/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.commands;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.MathCommand;

/**
 * @author Sascha Schneiders
 */
public class MathRowCommand extends MathCommand {
    public MathRowCommand() {
        setMathCommandName("row");
    }
    @Override
    public void convert(MathExpressionSymbol mathExpressionSymbol, BluePrint bluePrint) {
        //do nothing
    }
}
