/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant;

import org.jscience.mathematics.number.Rational;

import javax.measure.unit.Unit;

/**
 * Represents a constant SIUnit which is used for constant connectors
 */
public class EMAConstantSIUnit extends EMAConstantValue {
    public Unit unit;

    public EMAConstantSIUnit(Rational number, Unit unit) {
        super(number);
        this.unit = unit;
    }

    public Unit getUnit() {
        return unit;
    }

    @Override
    public boolean isSIUnit() {
        return true;
    }

    public Rational getRational() {
        return (Rational) value;
    }

    /**
     * Does not return the unit, just the value
     *
     * @return
     */
    @Override
    public String getValueAsString() {
        String result = "";
        if (getRational().getDivisor().intValue() == 1) {
            result += getRational().intValue();
        } else {
            result += getRational().doubleValue();
        }
        return result;
    }
}
