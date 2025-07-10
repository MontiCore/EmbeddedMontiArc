/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.streamunits._symboltable;

import de.monticore.lang.monticar.Utils;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import org.jscience.mathematics.number.Rational;

/**
 */
public class StreamValuePrecision implements IStreamValue {
    protected Object value;
    protected Object precision;

    public StreamValuePrecision() {

    }

    public StreamValuePrecision(Object value) {
        this.value = value;
    }

    public StreamValuePrecision(Object value, Object precision) {
        this.value = value;
        this.precision = precision;
    }

    public Object getValue() {
        return value;
    }

    public void setValue(Object value) {
        this.value = value;
    }

    public Object getPrecision() {
        return precision;
    }

    public void setPrecision(Object precision) {
        this.precision = precision;
    }

    public boolean hasPrecision() {
        return precision != null;
    }

    @Override
    public String toString() {
        String valueString = value.toString();
        if (value instanceof Rational) {
            Rational rational = (Rational) value;
            valueString = Utils.getRationalString(rational);
        } else if (value instanceof ASTNumberWithUnit) {
            ASTNumberWithUnit NumberWithUnit = (ASTNumberWithUnit) value;
            if (NumberWithUnit.getNumber().isPresent()) {
                valueString = Double.toString(NumberWithUnit.getNumber().get());
            }
        }


        if (precision != null) {
            String precisionString = precision.toString();
            return valueString + " +/- " + precisionString;
        }
        return valueString;
    }

    @Override
    public boolean isStreamValuePrecision() {
        return true;
    }

    @Override
    public boolean isStreamValueDontCare() {
        return value.equals("-");
    }

    @Override
    public void visit(NamedStreamUnitsSymbol streamUnitsSymbol) {
        streamUnitsSymbol.add(this);
    }
}
