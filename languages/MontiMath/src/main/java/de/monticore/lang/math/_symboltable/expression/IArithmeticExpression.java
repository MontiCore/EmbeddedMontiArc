/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

/**
 */
public interface IArithmeticExpression {
    void setLeftExpression(MathExpressionSymbol mathExpressionSymbol);
    void setRightExpression(MathExpressionSymbol mathExpressionSymbol);

    MathExpressionSymbol getLeftExpression();
    MathExpressionSymbol getRightExpression();

    void setOperator(String operator);
    String getOperator();
}
