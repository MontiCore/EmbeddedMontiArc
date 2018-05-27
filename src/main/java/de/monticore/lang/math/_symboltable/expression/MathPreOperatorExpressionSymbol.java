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
package de.monticore.lang.math._symboltable.expression;

/**
 * @author Sascha Schneiders
 */
public class MathPreOperatorExpressionSymbol extends MathExpressionSymbol {
    protected MathExpressionSymbol mathExpressionSymbol;
    protected String operator;

    public MathExpressionSymbol getMathExpressionSymbol() {
        return mathExpressionSymbol;
    }

    public void setMathExpressionSymbol(MathExpressionSymbol mathExpressionSymbol) {
        this.mathExpressionSymbol = mathExpressionSymbol;
    }

    public String getOperator() {
        return operator;
    }

    public void setOperator(String operator) {
        this.operator = operator;
    }

    @Override
    public boolean isPreOperatorExpression() {
        return true;
    }

    @Override
    public String getTextualRepresentation() {
        return operator + mathExpressionSymbol.getTextualRepresentation();
    }

    /* TODO return negated expression
    @Override
    public MathExpressionSymbol getRealMathExpressionSymbol(){
        return mathExpressionSymbol;
    }*/
}
