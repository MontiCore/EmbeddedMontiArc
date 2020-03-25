/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

import javax.measure.unit.Unit;

/**
 *
 * An abstract type for a math value
 */
public interface MathValue {
    /**
     * format the math value to a string
     *
     * @return string representation of the math value
     */
    public String toString();

    /**
     * get the unit from the math value
     *
     * @return unit of the math value
     */
    public Unit getUnit();

    /**
     * set the unit for the math value
     *
     * @param unit (jscience unit)
     */
    public void setUnit(Unit unit);

    /**
     * check if there is a incompatible unit for this math value/expression
     *
     * @return TRUE if is incompatible, otherwise FALSE
     */
    public boolean isIncompUnit();

    /**
     * check and set if the math value has incompatible units
     *
     * @param x set TRUE if the math value/expression has incompatible units, otherwise FALSE
     */
    public void setIncompUnit(boolean x);

}
