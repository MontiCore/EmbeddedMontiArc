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

import de.monticore.lang.math._symboltable.expression.IArithmeticExpression;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

/**
 * @author Sascha Schneiders
 */
public class MathMatrixArithmeticExpressionSymbol extends MathMatrixExpressionSymbol implements IArithmeticExpression {

    protected MathExpressionSymbol rightExpression;
    protected MathExpressionSymbol leftExpression;

    protected String mathOperator;

    public MathMatrixArithmeticExpressionSymbol() {
        super();
    }

    public MathExpressionSymbol getRightExpression() {
        return rightExpression;
    }

    public void setRightExpression(MathExpressionSymbol rightExpression) {
        this.rightExpression = rightExpression;
    }

    public MathExpressionSymbol getLeftExpression() {
        return leftExpression;
    }

    public void setLeftExpression(MathExpressionSymbol leftExpression) {
        this.leftExpression = leftExpression;
    }

    public String getMathOperator() {
        return mathOperator;
    }

    public void setMathOperator(String mathOperator) {
        this.mathOperator = mathOperator;
    }

    public String getOperator() {
        return getMathOperator();
    }

    public void setOperator(String operator) {
        setMathOperator(operator);
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";
        if (leftExpression != null)
            result += leftExpression.getTextualRepresentation();
        result += mathOperator;
        if (rightExpression != null)
            result += rightExpression.getTextualRepresentation();

        return result;
    }

    @Override
    public boolean isMatrixArithmeticExpression() {
        return true;
    }

    public boolean isAdditionOperator() {
        return mathOperator.equals("+");
    }


    public boolean isSubtractionOperator() {
        return mathOperator.equals("-");
    }


    public boolean isMultiplicationOperator() {
        return mathOperator.equals("*");
    }

    public boolean isDivisionOperator() {
        return mathOperator.equals("/");
    }


    public boolean isModuloOperator() {
        return mathOperator.equals("%");
    }


    public boolean isPowerOfOperator() {
        return mathOperator.equals("^");
    }

}
