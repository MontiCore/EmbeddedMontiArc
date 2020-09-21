/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

import java.util.List;

public class EventBracketExpressionSymbol extends EventExpressionSymbol {

    protected EventExpressionSymbol innerExpression;

    public EventBracketExpressionSymbol(EventExpressionSymbol innerExpression){
        super();
        this.innerExpression = innerExpression;
    }

    @Override
    public String getTextualRepresentation() {
        return "("+this.innerExpression.getTextualRepresentation()+")";
    }

    @Override
    public String toString() {
        return "de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventBracketExpressionSymbol("+this.innerExpression.toString()+")";
    }

    public EventExpressionSymbol getInnerExpression() {
        return innerExpression;
    }

    public void setInnerExpression(EventExpressionSymbol innerExpression) {
        this.innerExpression = innerExpression;
    }

    @Override
    public EventExpressionSymbol expand() {
        return new EventBracketExpressionSymbol(innerExpression.expand());
    }

    @Override
    public boolean hasConnectSymbol(){
        return this.innerExpression.hasConnectSymbol();
    }

    @Override
    public void getConnectPortNames(List<String> names){
        this.innerExpression.getConnectPortNames(names);
    }

    @Override
    public boolean hasFreeSymbol() {
        return this.innerExpression.hasFreeSymbol();
    }

    @Override
    public void getFreePortNames(List<String> names) {
        this.innerExpression.getFreePortNames(names);
    }
}
