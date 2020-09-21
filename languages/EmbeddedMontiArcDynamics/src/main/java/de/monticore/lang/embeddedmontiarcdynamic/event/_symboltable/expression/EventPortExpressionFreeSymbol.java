/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

import java.util.List;
import java.util.Optional;

public class EventPortExpressionFreeSymbol extends EventPortExpression {
    protected Optional<String> componentName;

    public EventPortExpressionFreeSymbol(){
        super();
    }

    @Override
    public String getTextualRepresentation() {

        return super.getTextualRepresentation()+ "::free";
    }

    @Override
    public EventPortExpression expand() {
        EventPortExpressionFreeSymbol cs = new EventPortExpressionFreeSymbol();
        this.expand_setProperties(cs);
        return cs;
    }

    @Override
    public boolean hasFreeSymbol() {
        return true;
    }

    @Override
    public void getFreePortNames(List<String> names) {
        if(!names.contains(this.getName())) {
            names.add(this.getName());
        }
    }
}
