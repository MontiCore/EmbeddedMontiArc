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
package de.monticore.lang.math.math._symboltable;

import javax.measure.unit.Unit;

/**
 * @author math-group
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
