/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues;

import de.monticore.lang.monticar.interfaces.TextualExpression;
import de.monticore.symboltable.CommonSymbol;

public class PortValueSymbol extends CommonSymbol implements TextualExpression {

    public static PortValueSymbolKIND KIND = new PortValueSymbolKIND();

    public PortValueSymbol(){
        super("", KIND);
    }

    public PortValueSymbol(String name){
        super(name,KIND);
    }


    @Override
    public String getTextualRepresentation() {
        return "";
    }

    @Override
    public String toString() {
        return this.getClass().getSimpleName()+"["+getTextualRepresentation()+"]";
    }

    public PortValueSymbol getForIndex(int index){
        return this;
    }
}
