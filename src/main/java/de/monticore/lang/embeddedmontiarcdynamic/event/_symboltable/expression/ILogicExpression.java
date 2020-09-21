/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

public interface ILogicExpression {

    void setOperator(String operator);
    String getOperator();

    void setLeftExpression(EventExpressionSymbol expression);
    EventExpressionSymbol getLeftExpression();

    void setRightExpression(EventExpressionSymbol expression);
    EventExpressionSymbol getRightExpression();


}
