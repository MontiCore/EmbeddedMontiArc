/**
 * ******************************************************************************
 * MontiCAR Modeling Family, www.se-rwth.de
 * Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 * All rights reserved.
 * <p>
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * <p>
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.streamunits._symboltable;

import de.monticore.lang.monticar.Utils;
import de.monticore.lang.numberunit._ast.ASTUnitNumber;
import org.jscience.mathematics.number.Rational;

/**
 * @author Sascha Schneiders
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
        } else if (value instanceof ASTUnitNumber) {
            ASTUnitNumber unitNumber = (ASTUnitNumber) value;
            if (unitNumber.getNumber().isPresent()) {
                valueString = Utils.getRationalString(unitNumber.getNumber().get());
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
