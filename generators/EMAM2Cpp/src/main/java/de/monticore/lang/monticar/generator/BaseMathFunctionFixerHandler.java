/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.pattern.BaseChainOfResponsibility;
import de.se_rwth.commons.logging.Log;

public abstract class BaseMathFunctionFixerHandler extends BaseChainOfResponsibility<BaseMathFunctionFixerHandler> {

    protected abstract boolean canFixMathSymbol(MathExpressionSymbol symbol);

    protected abstract void doFixMathFunction(MathExpressionSymbol symbol, EMAMBluePrintCPP bluePrintCPP);

    private void handleFixMathFunction(MathExpressionSymbol symbol, EMAMBluePrintCPP bluePrintCPP) {
        if (canFixMathSymbol(symbol)) {
            doFixMathFunction(symbol, bluePrintCPP);
        } else if (getSuccessor() != null) {
            ((BaseMathFunctionFixerHandler) getSuccessor()).handleFixMathFunction(symbol, bluePrintCPP);
        } else {
            Log.info(symbol.getTextualRepresentation(), "Symbol:");
            Log.debug(getRole(), "Case not handled!");
        }
    }

    public void chainHandleFixMathFunction(MathExpressionSymbol symbol, EMAMBluePrintCPP bluePrintCPP) {
        ((BaseMathFunctionFixerHandler) getChainStart()).handleFixMathFunction(symbol, bluePrintCPP);
    }

}
