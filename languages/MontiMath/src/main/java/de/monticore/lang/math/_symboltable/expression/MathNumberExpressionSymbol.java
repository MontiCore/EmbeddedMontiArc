/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

import de.monticore.lang.math._symboltable.JSValue;
import org.jscience.mathematics.number.Rational;

/**
 */
public class MathNumberExpressionSymbol extends MathValueExpressionSymbol {
    JSValue value;

    public MathNumberExpressionSymbol() {
        super();
    }

    public MathNumberExpressionSymbol(Rational number) {
        super();
        value = new JSValue(number);
    }

    public void setValue(JSValue value) {
        this.value = value;
    }

    public JSValue getValue() {
        return value;
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";
        if (value.getImagNumber().isPresent())
            return value.toString();
        if (value.getRealNumber().getDivisor().intValue() == 1) {
            result += value.getRealNumber().getDividend().intValue();
        } else {
            result += value.getRealNumber().doubleValue();
        }
        return result;
    }

    @Override
    public boolean isNumberExpression() {
        return true;
    }
}
