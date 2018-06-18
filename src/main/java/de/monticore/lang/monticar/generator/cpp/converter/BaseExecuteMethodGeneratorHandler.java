package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.List;

public abstract class BaseExecuteMethodGeneratorHandler {

    private BaseExecuteMethodGeneratorHandler successor = null;

    public BaseExecuteMethodGeneratorHandler getSuccessor() {
        return successor;
    }

    public void setSuccessor(BaseExecuteMethodGeneratorHandler successor) {
        this.successor = successor;
    }

    protected abstract boolean canHandleSymbol(MathExpressionSymbol symbol);

    protected abstract String doGenerateExecuteCode(MathExpressionSymbol symbol, List<String> includeStrings);

    public abstract String getRole();

    public String handleGenerateExecuteCode(MathExpressionSymbol symbol, List<String> includeStrings) {
        String result = "";
        if (canHandleSymbol(symbol)) {
            result = doGenerateExecuteCode(symbol, includeStrings);
        } else if (successor != null) {
            result = successor.handleGenerateExecuteCode(symbol, includeStrings);
        } else {
            Log.info(symbol.getTextualRepresentation(), "Symbol:");
            Log.debug(getRole(), "Case not handled!");
        }
        return result;
    }

}
