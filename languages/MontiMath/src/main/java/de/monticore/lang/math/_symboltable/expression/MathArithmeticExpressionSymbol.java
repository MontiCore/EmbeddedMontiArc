/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

/**
 */
public class MathArithmeticExpressionSymbol extends MathExpressionSymbol implements IArithmeticExpression {
    protected MathExpressionSymbol rightExpression;
    protected MathExpressionSymbol leftExpression;

    protected String mathOperator;

    public MathArithmeticExpressionSymbol() {
        super();
    }

    public MathExpressionSymbol getRightExpression() {
        return rightExpression;
    }

    @Override
    public void setOperator(String operator) {
        setMathOperator(operator);
    }

    @Override
    public String getOperator() {
        return getMathOperator();
    }

    public void setRightExpression(MathExpressionSymbol rightExpression) {
        this.rightExpression = rightExpression;
    }

    public MathExpressionSymbol getLeftExpression() {
        return leftExpression;
    }

    public void setLeftExpression(MathExpressionSymbol leftExpression) {
        this.leftExpression = leftExpression;
    }

    public String getMathOperator() {
        return mathOperator;
    }

    public void setMathOperator(String mathOperator) {
        this.mathOperator = mathOperator;
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";
        if (leftExpression != null)
            result += leftExpression.getTextualRepresentation();
        result += mathOperator;
        if (rightExpression != null)
            result += rightExpression.getTextualRepresentation();

        return result;
    }

    @Override
    public boolean isArithmeticExpression() {
        return true;
    }

    public boolean isAdditionOperator() {
        return mathOperator.equals("+");
    }


    public boolean isSubtractionOperator() {
        return mathOperator.equals("-");
    }


    public boolean isMultiplicationOperator() {
        return mathOperator.equals("*");
    }

    public boolean isDivisionOperator() {
        return mathOperator.equals("/");
    }


    public boolean isModuloOperator() {
        return mathOperator.equals("%");
    }


    public boolean isPowerOfOperator() {
        return mathOperator.equals("^");
    }

    public boolean isIncreaseByOneOperator() {
        return mathOperator.equals("++");
    }

    public boolean isDecreaseByOneOperator() {
        return mathOperator.equals("--");
    }


}
