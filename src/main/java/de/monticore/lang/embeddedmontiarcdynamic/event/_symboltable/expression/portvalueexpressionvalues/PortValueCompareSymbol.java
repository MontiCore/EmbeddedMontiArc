/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues;

public class PortValueCompareSymbol extends PortValueSymbol {

    protected String operator;
    protected PortValueInputSymbol compareValue;

    public PortValueCompareSymbol(String operator, PortValueInputSymbol compareValue){
        super();
        this.operator = operator;
        this.compareValue = compareValue;
    }

    public PortValueInputSymbol getCompareValue() {
        return compareValue;
    }

    public String getOperator() {
        return operator;
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";
        if(!this.operator.equalsIgnoreCase("==")){
            result += this.operator;
        }

        result += this.compareValue.getTextualRepresentation();

        return result;
    }

}
