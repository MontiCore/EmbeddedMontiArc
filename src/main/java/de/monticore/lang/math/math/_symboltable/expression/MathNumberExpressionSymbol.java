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
package de.monticore.lang.math.math._symboltable.expression;

import de.monticore.lang.math.math._symboltable.JSValue;
import org.jscience.mathematics.number.Rational;

/**
 * @author Sascha Schneiders
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
