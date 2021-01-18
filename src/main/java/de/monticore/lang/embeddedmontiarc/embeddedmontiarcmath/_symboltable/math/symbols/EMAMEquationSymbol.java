/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

public class EMAMEquationSymbol extends MathExpressionSymbol {

    protected MathExpressionSymbol leftExpression;

    protected MathExpressionSymbol rightExpression;

    public EMAMEquationSymbol() {
        super();
    }

    @Override
    public String getTextualRepresentation() {
        return leftExpression.getTextualRepresentation() + "==" + rightExpression.getTextualRepresentation();
    }

    public MathExpressionSymbol getLeftExpression() {
        return leftExpression;
    }

    public void setLeftExpression(MathExpressionSymbol leftExpression) {
        this.leftExpression = leftExpression;
    }

    public MathExpressionSymbol getRightExpression() {
        return rightExpression;
    }

    public void setRightExpression(MathExpressionSymbol rightExpression) {
        this.rightExpression = rightExpression;
    }

}
