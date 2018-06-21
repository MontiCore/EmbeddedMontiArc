package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.pattern.BaseChainOfResponsibility;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 * Implements the chain-of-responsibility pattern
 *
 * @author Christoph Richter
 */
public abstract class BaseExecuteMethodGeneratorHandler extends BaseChainOfResponsibility<BaseExecuteMethodGeneratorHandler> {

    protected abstract boolean canHandleSymbol(MathExpressionSymbol symbol);

    protected abstract String doGenerateExecuteCode(MathExpressionSymbol symbol, List<String> includeStrings);

    public String handleGenerateExecuteCode(MathExpressionSymbol symbol, List<String> includeStrings) {
        String result = "";
        if (canHandleSymbol(symbol)) {
            result = doGenerateExecuteCode(symbol, includeStrings);
        } else if (getSuccessor() != null) {
            result = getSuccessor().handleGenerateExecuteCode(symbol, includeStrings);
        } else {
            Log.info(symbol.getTextualRepresentation(), "Symbol:");
            Log.debug(getRole(), "Case not handled!");
        }
        return result;
    }

}
