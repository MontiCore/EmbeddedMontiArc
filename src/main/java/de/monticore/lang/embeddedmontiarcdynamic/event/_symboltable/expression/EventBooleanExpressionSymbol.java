/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

public class EventBooleanExpressionSymbol extends EventExpressionSymbol {

    protected boolean booleanValue = false;

    public EventBooleanExpressionSymbol(boolean value){
        super();
        booleanValue = value;
    }

    @Override
    public String getTextualRepresentation() {
        return booleanValue ? "true" : "false";
    }

    @Override
    public String toString(){
        return "de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventBooleanExpressionSymbol("+booleanValue+")";
    }

    @Override
    public EventExpressionSymbol expand() {
        return new EventBooleanExpressionSymbol(booleanValue);
    }

    public boolean getBooleanValue() {
        return booleanValue;
    }
}
