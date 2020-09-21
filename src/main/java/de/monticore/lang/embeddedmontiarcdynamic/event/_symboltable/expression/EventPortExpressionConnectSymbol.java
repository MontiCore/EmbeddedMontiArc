/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

import java.util.List;
import java.util.Optional;

public class EventPortExpressionConnectSymbol extends EventPortExpression {


    public EventPortExpressionConnectSymbol(){
        super();
    }

    @Override
    public String getTextualRepresentation() {
        return super.getTextualRepresentation() + "::connect";
    }

    @Override
    public EventExpressionSymbol expand() {
        EventPortExpressionConnectSymbol cs = new EventPortExpressionConnectSymbol();
        this.expand_setProperties(cs);
        return cs;
    }

    @Override
    public boolean hasConnectSymbol() {
        return true;
    }

    @Override
    public void getConnectPortNames(List<String> names){
        if(!names.contains(this.getName())){
            names.add(this.getName());
        }
    }
}
