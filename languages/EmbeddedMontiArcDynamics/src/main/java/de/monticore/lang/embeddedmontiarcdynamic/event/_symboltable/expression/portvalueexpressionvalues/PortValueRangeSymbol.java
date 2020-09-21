/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues;

public class PortValueRangeSymbol extends PortValueSymbol {

    protected PortValueInputSymbol lowerBound;

    protected PortValueInputSymbol upperBound ;

    public PortValueRangeSymbol(PortValueInputSymbol lower, PortValueInputSymbol upper){
        super();
        this.lowerBound = lower;
        this.upperBound = upper;
    }

    public PortValueInputSymbol getLowerBound(){
        return lowerBound;
    }

    public PortValueInputSymbol getUpperBound(){
        return upperBound;
    }

    @Override
    public String getTextualRepresentation() {
        return "("+lowerBound.getTextualRepresentation()+":"+upperBound.getTextualRepresentation()+")";
    }

}
