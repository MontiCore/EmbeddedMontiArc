/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValueSymbol;
import de.monticore.symboltable.Symbol;

public class EventPortExpressionValueSymbol extends EventPortExpression implements Symbol {


    protected PortValueSymbol portValue;


    public EventPortExpressionValueSymbol(){
        super();
    }
    public PortValueSymbol getPortValue() {
        return portValue;
    }

    public void setPortValue(PortValueSymbol portValue) {
        this.portValue = portValue;
    }

    @Override
    public String getTextualRepresentation() {
        return super.getTextualRepresentation() + "::value("+this.portValue.getTextualRepresentation()+")";
    }

    @Override
    public EventExpressionSymbol expand() {
        EventPortExpressionValueSymbol epevs = new EventPortExpressionValueSymbol();
        this.expand_setProperties(epevs);

        epevs.setPortValue(this.portValue);

        return epevs;
    }
}
