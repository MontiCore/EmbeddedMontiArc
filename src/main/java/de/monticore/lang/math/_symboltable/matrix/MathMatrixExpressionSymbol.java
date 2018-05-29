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
package de.monticore.lang.math._symboltable.matrix;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

/**
 *
 */
public abstract class MathMatrixExpressionSymbol extends MathExpressionSymbol {
    public MathMatrixExpressionSymbol() {
        super();
    }

    public MathMatrixExpressionSymbol(String name) {
        super(name);
    }

    @Override
    public boolean isMatrixExpression() {
        return true;
    }

    public boolean isMatrixVectorExpression() {
        return false;
    }

    public boolean isMatrixAccessExpression() {
        return false;
    }

    public boolean isMatrixArithmeticExpression() {
        return false;
    }

    public boolean isMatrixNameExpression() {
        return false;
    }

    public boolean isMatrixPreOperatorSymbol() {
        return false;
    }
}
