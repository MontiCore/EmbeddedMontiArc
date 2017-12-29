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

/**
 * @author Sascha Schneiders
 */
public class MathCompareExpressionSymbol extends MathExpressionSymbol implements IArithmeticExpression {
    protected String compareOperator;
    protected MathExpressionSymbol leftExpression;
    protected MathExpressionSymbol rightExpression;

    public MathCompareExpressionSymbol() {

    }

    public String getCompareOperator() {
        return compareOperator;
    }

    public void setCompareOperator(String compareOperator) {
        this.compareOperator = compareOperator;
    }

    public MathExpressionSymbol getLeftExpression() {
        return leftExpression;
    }

    public void setLeftExpression(MathExpressionSymbol leftExpression) {
        this.leftExpression = leftExpression;
    }

    public MathExpressionSymbol getRightExpression() {
        return rightExpression;
    }

    @Override
    public void setOperator(String operator) {
        setCompareOperator(operator);
    }

    @Override
    public String getOperator() {
        return getCompareOperator();
    }

    public void setRightExpression(MathExpressionSymbol rightExpression) {
        this.rightExpression = rightExpression;
    }

    @Override
    public String getTextualRepresentation() {
        return leftExpression.getTextualRepresentation() + compareOperator + rightExpression.getTextualRepresentation();
    }

    @Override
    public boolean isCompareExpression() {
        return true;
    }
}
