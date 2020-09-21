/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

import de.monticore.lang.monticar.interfaces.TextualExpression;
import de.monticore.symboltable.CommonSymbol;
import org.w3c.dom.events.EventException;

import java.util.List;

public class EventExpressionSymbol extends CommonSymbol implements TextualExpression {

    public static EventExpressionSymbolKIND KIND = new EventExpressionSymbolKIND();

    public EventExpressionSymbol(){
        super("", KIND);
    }

    public EventExpressionSymbol(String name){
        super(name,KIND);
    }

    @Override
    public String getTextualRepresentation() {
        return "";
    }

    @Override
    public String toString() {
        return this.getClass().getSimpleName()+"["+this.getTextualRepresentation()+"]";
    }


    public EventExpressionSymbol expand(){
        return new EventExpressionSymbol(this.getName());
    }

    public boolean hasConnectSymbol(){
        return false;
    }

    public void getConnectPortNames(List<String> names){
        return;
    }

    public boolean hasFreeSymbol(){
        return false;
    }

    public void getFreePortNames(List<String> names) {
        return;
    }
}
