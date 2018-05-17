/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.math._symboltable;

import org.jscience.mathematics.number.Rational;

import javax.measure.unit.Unit;
import java.util.Optional;

/**
 * @author math-group
 *         <p>
 *         JScience Value Type
 */
public class JSValue implements MathValue {
    /**
     * JScience concrete value
     */
    private Rational realNumber;
    /**
     * imaginary part for numbers from C
     */
    private Optional<Rational> imagNumber = Optional.empty();
    /**
     * JScience unit
     */
    private Unit unit;

    /**
     * flag for incompatible unit
     */
    private boolean incompUnit = false;

    /**
     * initialize every JSValue with the base value (Rational.ONE) {@link this.number}
     * and the base unit (Unit.ONE) {@link this.unit}
     */
    public JSValue() {
        this.realNumber = Rational.ONE;
        this.unit = Unit.ONE;
    }

    /**
     * initialize every JSValue with the value given by the argument
     * and the base unit (Unit.ONE) {@link this.unit}
     *
     * @param number the new value
     */
    public JSValue(Rational number) {
        this.realNumber = number;
        this.unit = Unit.ONE;
    }

    /**
     * initialize every JSValue with the unit given by the argument
     * and the base value (Rational.ONE) {@link this.number}
     *
     * @param unit the new unit
     */
    public JSValue(Unit unit) {
        this.unit = unit;
        this.realNumber = Rational.ONE;
    }

    /**
     * initialize a JSValue with an imaginary part, so a number from C
     */
    public JSValue(Rational realNumber, Rational imagNumber) {
        this.realNumber = realNumber;
        this.imagNumber = Optional.of(imagNumber);
        this.unit = Unit.ONE;
    }

    /**
     * create a new JScience value given by the arguments
     *
     * @param number new value
     * @param unit   new unit
     */
    public JSValue(Rational number, Unit unit) {
        this.realNumber = number;
        this.unit = unit;
    }

    /**
     * create a new JSValue with an imaginary part, so a number from C and add a unit
     */
    public JSValue(Rational realNumber, Rational imagNumber, Unit unit) {
        this.realNumber = realNumber;
        this.imagNumber = Optional.of(imagNumber);
        this.unit = unit;
    }

    /**
     * @return get the unit of the JScience value
     */
    public Unit getUnit() {
        return this.unit;
    }

    /**
     * set a new unit
     *
     * @param unit (jscience unit)
     */
    public void setUnit(Unit unit) {
        this.unit = unit;
    }

    /**
     * get the value of this JScience type
     *
     * @return value
     */
    public Rational getRealNumber() {
        return this.realNumber;
    }

    /**
     * get the imaginary value part of this JScience type
     *
     * @return value
     */
    public Optional<Rational> getImagNumber() {
        return this.imagNumber;
    }

    /**
     * set a new value for this JScience type
     *
     * @param number new value
     */
    public void setImagNumber(Rational number) {
        this.imagNumber = Optional.of(number);
    }

    /**
     * set a new value for this JScience type
     *
     * @param number new value
     */
    public void setRealNumber(Rational number) {
        this.realNumber = number;
    }

    public boolean isComplex() {
        return imagNumber.isPresent();
    }

    /**
     * check if the unit is incompatible
     *
     * @return TRUE, if the units are incompatible, otherwise FALSE
     */
    @Override
    public boolean isIncompUnit() {
        return incompUnit;
    }

    /**
     * set the flag
     *
     * @param x set TRUE if the math value/expression has incompatible units, otherwise FALSE
     */
    @Override
    public void setIncompUnit(boolean x) {
        incompUnit = x;
    }

    @Override
    public String toString() {
        String output = "";
        output += realNumber.toText();
        if (imagNumber.isPresent()) {
            output += "+" + imagNumber.get().toText() + "i ";
        }
        return output + " " + unit.toString();
    }

    public boolean isJSValue() {
        return true;
    }
}
