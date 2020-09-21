/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues;

import java.util.ArrayList;
import java.util.List;

public class PortValuesMatrixSymbol extends PortValueSymbol {

    protected List<PortValuesArraySymbol> valuesMatrix;

    public PortValuesMatrixSymbol(){
        super();
        valuesMatrix = new ArrayList<>();
    }

    public void add(PortValuesArraySymbol value){
        this.valuesMatrix.add(value);
    }

    @Override
    public String getTextualRepresentation() {
        StringBuilder sb = new StringBuilder();
        for(int i = 0; i < valuesMatrix.size(); ++i){
            sb.append(valuesMatrix.get(i).getInnerTextualRepresentation());
            if(i < (valuesMatrix.size()-1)){
                sb.append(";");
            }
        }
        return "[" + sb.toString() + "]";
    }

    @Override
    public PortValueSymbol getForIndex(int index) {
        return valuesMatrix.get(index % valuesMatrix.size());
    }

    public int size(){
        return valuesMatrix.size();
    }

    public PortValuesArraySymbol getArraySymbolForIndex(int index){
        return valuesMatrix.get(index % valuesMatrix.size());
    }
}
