/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.matrix;

import de.monticore.lang.math._symboltable.expression.IArithmeticExpression;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

/**
 */
public class MathMatrixArithmeticExpressionSymbol extends MathMatrixExpressionSymbol implements IArithmeticExpression {

    protected MathExpressionSymbol rightExpression;
    protected MathExpressionSymbol leftExpression;

    protected String mathOperator;

    public MathMatrixArithmeticExpressionSymbol() {
        super();
    }

    public MathExpressionSymbol getRightExpression() {
        return rightExpression;
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

    public String getOperator() {
        return getMathOperator();
    }

    public void setOperator(String operator) {
        setMathOperator(operator);
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
    public boolean isMatrixArithmeticExpression() {
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

}
