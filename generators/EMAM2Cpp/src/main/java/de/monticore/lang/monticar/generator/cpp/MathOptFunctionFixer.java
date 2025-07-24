/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;
import de.monticore.lang.monticar.generator.BaseMathFunctionFixerHandler;

/**
 * fixes math optimization functions
 *
 */
public class MathOptFunctionFixer extends BaseMathFunctionFixerHandler {

    @Override
    protected boolean canFixMathSymbol(MathExpressionSymbol symbol) {
        boolean canHandle = false;
        if (symbol instanceof MathOptimizationStatementSymbol)
            canHandle = true;
        return canHandle;
    }

    @Override
    protected void doFixMathFunction(MathExpressionSymbol symbol, EMAMBluePrintCPP bluePrintCPP) {
        // nothing to fix here
    }

    @Override
    public String getRole() {
        return "MathOptFunctionFixer";
    }
}
