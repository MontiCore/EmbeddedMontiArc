/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

/**
 */
public class MathCompareExpressionSymbol extends MathExpressionSymbol implements IArithmeticExpression {
    protected String compareOperator;
    protected MathExpressionSymbol leftExpression;
    protected MathExpressionSymbol rightExpression;

    public MathCompareExpressionSymbol() {

    }

    public String getCompareOperator() {
        return compareOperator;
    }

    public void setCompareOperator(String compareOperator) {
        this.compareOperator = compareOperator;
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

    @Override
    public void setOperator(String operator) {
        setCompareOperator(operator);
    }

    @Override
    public String getOperator() {
        return getCompareOperator();
    }

    public void setRightExpression(MathExpressionSymbol rightExpression) {
        this.rightExpression = rightExpression;
    }

    @Override
    public String getTextualRepresentation() {
        return leftExpression.getTextualRepresentation() + compareOperator + rightExpression.getTextualRepresentation();
    }

    @Override
    public boolean isCompareExpression() {
        return true;
    }
}
