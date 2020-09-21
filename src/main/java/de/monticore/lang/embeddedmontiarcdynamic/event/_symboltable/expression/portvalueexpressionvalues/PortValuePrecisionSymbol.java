/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues;

import java.util.Optional;

public class PortValuePrecisionSymbol extends PortValueSymbol {

    protected PortValueInputSymbol value;
    protected Optional<PortValueInputSymbol> precision = Optional.empty();

    public PortValuePrecisionSymbol(PortValueInputSymbol value){
        super();
        this.value = value;
    }

    public PortValuePrecisionSymbol(PortValueInputSymbol value, PortValueInputSymbol precision) {
        super();
        this.value = value;
        this.precision = Optional.of(precision);
    }

    public boolean hasPrecision(){
        return this.precision.isPresent();
    }

    public void setPrecision(PortValueInputSymbol precision){
        this.precision = Optional.of(precision);
    }

    public PortValueInputSymbol getPrecision() {
        return precision.get();
    }

    @Override
    public String getTextualRepresentation() {

        String result = value.getTextualRepresentation();


        if(this.hasPrecision()){
            result += ("+/-"+precision.get().getTextualRepresentation());
        }

        return result;
    }

    public PortValueInputSymbol getValue() {
        return value;
    }
}
