/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues;

import java.util.ArrayList;
import java.util.List;

public class PortValuesArraySymbol extends PortValueSymbol{

    protected List<PortValueSymbol> valuesArray;

    public PortValuesArraySymbol(){
        super();
        valuesArray = new ArrayList<>();
    }

    public void add(PortValueSymbol value){
        valuesArray.add(value);
    }

    @Override
    public String getTextualRepresentation() {
        return "["+this.getInnerTextualRepresentation()+"]";
    }

    public String getInnerTextualRepresentation(){
        String result = "";
        for(int i = 0; i < this.valuesArray.size(); ++i){
            result += this.valuesArray.get(i).getTextualRepresentation();
            if(i < (this.valuesArray.size()-1)){
                result += ",";
            }
        }
        return result;
    }

    public int size(){
        return valuesArray.size();
    }

    @Override
    public PortValueSymbol getForIndex(int index) {
        return valuesArray.get(index % valuesArray.size());
    }
}
