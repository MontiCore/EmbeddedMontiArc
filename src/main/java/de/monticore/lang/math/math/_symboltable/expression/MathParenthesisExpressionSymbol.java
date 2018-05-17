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

import de.se_rwth.commons.logging.Log;

/**
 * @author Sascha Schneiders
 */
public class MathParenthesisExpressionSymbol extends MathExpressionSymbol {
    protected MathExpressionSymbol mathExpressionSymbol;

    public MathParenthesisExpressionSymbol() {

    }

    public MathParenthesisExpressionSymbol(MathExpressionSymbol mathExpressionSymbol) {
        this.mathExpressionSymbol = mathExpressionSymbol;
    }

    public MathExpressionSymbol getMathExpressionSymbol() {
        return mathExpressionSymbol;
    }

    public void setMathExpressionSymbol(MathExpressionSymbol mathExpressionSymbol) {
        this.mathExpressionSymbol = mathExpressionSymbol;
    }

    @Override
    public boolean isParenthesisExpression() {
        return true;
    }

    @Override
    public String getTextualRepresentation() {
        if (mathExpressionSymbol == null)
            return "(null)";
        return "(" + mathExpressionSymbol.getTextualRepresentation() + ")";
    }

    @Override
    public MathExpressionSymbol getRealMathExpressionSymbol() {
        if (mathExpressionSymbol == null) {
            Log.info("MathExpressionSymbol is null", "useless parenthesis:");
        }

        return mathExpressionSymbol.getRealMathExpressionSymbol();
    }

    @Override
    public MathExpressionSymbol getAssignedMathExpressionSymbol() {
        return mathExpressionSymbol;
    }

}
