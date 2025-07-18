/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

import de.se_rwth.commons.logging.Log;

/**
 */
public class MathParenthesisExpressionSymbol extends MathExpressionSymbol {
    protected MathExpressionSymbol mathExpressionSymbol;

    public MathParenthesisExpressionSymbol() {

    }

    public MathParenthesisExpressionSymbol(MathExpressionSymbol mathExpressionSymbol) {
        this.mathExpressionSymbol = mathExpressionSymbol;
    }

    public MathExpressionSymbol getMathExpressionSymbol() {
        return mathExpressionSymbol;
    }

    public void setMathExpressionSymbol(MathExpressionSymbol mathExpressionSymbol) {
        this.mathExpressionSymbol = mathExpressionSymbol;
    }

    @Override
    public boolean isParenthesisExpression() {
        return true;
    }

    @Override
    public String getTextualRepresentation() {
        if (mathExpressionSymbol == null)
            return "(null)";
        return "(" + mathExpressionSymbol.getTextualRepresentation() + ")";
    }

    @Override
    public MathExpressionSymbol getRealMathExpressionSymbol() {
        if (mathExpressionSymbol == null) {
            Log.info("MathExpressionSymbol is null", "useless parenthesis:");
        }

        return mathExpressionSymbol.getRealMathExpressionSymbol();
    }

    @Override
    public MathExpressionSymbol getAssignedMathExpressionSymbol() {
        return mathExpressionSymbol;
    }

}
