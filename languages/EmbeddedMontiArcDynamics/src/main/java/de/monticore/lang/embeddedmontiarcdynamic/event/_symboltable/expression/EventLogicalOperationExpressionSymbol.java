/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

import java.util.List;

public class EventLogicalOperationExpressionSymbol extends EventExpressionSymbol implements ILogicExpression {

    protected EventExpressionSymbol leftExpression;
    protected EventExpressionSymbol rightExpression;
    protected String operator;

    public EventLogicalOperationExpressionSymbol(String operator, EventExpressionSymbol left, EventExpressionSymbol right){
        super();
        this.operator = operator;
        this.leftExpression = left;
        this.rightExpression = right;
    }


    @Override
    public void setOperator(String operator) {
        this.operator = operator;
    }

    @Override
    public String getOperator() {
        return this.operator;
    }

    @Override
    public void setLeftExpression(EventExpressionSymbol expression) {
        this.leftExpression = expression;
    }

    @Override
    public EventExpressionSymbol getLeftExpression() {
        return this.leftExpression;
    }

    @Override
    public void setRightExpression(EventExpressionSymbol expression) {
        this.rightExpression = expression;
    }

    @Override
    public EventExpressionSymbol getRightExpression() {
        return this.rightExpression;
    }

    @Override
    public String getTextualRepresentation() {

        String result = "";

        if(leftExpression != null){
            result += leftExpression.getTextualRepresentation();
        }

        result += operator;

        if(rightExpression != null){
            result += rightExpression.getTextualRepresentation();
        }

        return result;
    }

    @Override
    public String toString() {
        String l;
        if(leftExpression != null){
            l = leftExpression.toString();
        }else{
            l = "null";
        }
        String r;
        if(rightExpression != null){
            r = rightExpression.toString();
        }else{
            r = "null";
        }
        return "de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventLogicalOperationExpressionSymbol("+operator+")["+l+";"+r+"]";
    }

    @Override
    public EventExpressionSymbol expand() {
        if (leftExpression == null) {
            return new EventLogicalOperationExpressionSymbol(operator, null, rightExpression.expand());
        }
        return new EventLogicalOperationExpressionSymbol(operator, leftExpression.expand(), rightExpression.expand());
    }

    @Override
    public boolean hasConnectSymbol(){
        boolean result = false;
        if(leftExpression != null){
            result = leftExpression.hasConnectSymbol();
        }
        if(rightExpression != null){
            result = result || rightExpression.hasConnectSymbol();
        }
        return result;
    }

    @Override
    public void getConnectPortNames(List<String> names){
        if(leftExpression != null){
            leftExpression.getConnectPortNames(names);
        }
        if(rightExpression != null){
            rightExpression.getConnectPortNames(names);
        }
    }

    @Override
    public boolean hasFreeSymbol() {
        boolean result = false;
        if(leftExpression != null){
            result = leftExpression.hasFreeSymbol();
        }
        if(rightExpression != null){
            result = result || rightExpression.hasFreeSymbol();
        }
        return result;
    }

    @Override
    public void getFreePortNames(List<String> names) {
        if(leftExpression != null){
            leftExpression.getFreePortNames(names);
        }
        if(rightExpression != null){
            rightExpression.getFreePortNames(names);
        }
    }
}
