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

import de.monticore.lang.math._symboltable.MathAssignmentOperator;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;

/**
 * Symbol for setting/changing a MathValue
 *
 * @author Sascha Schneiders
 */
public class MathAssignmentExpressionSymbol extends MathExpressionSymbol {
    protected String nameOfMathValue;
    protected MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol;
    protected MathAssignmentOperator assignmentOperator;
    protected MathExpressionSymbol expressionSymbol;

    public String getNameOfMathValue() {
        return nameOfMathValue;
    }

    public void setNameOfMathValue(String nameOfMathValue) {
        this.nameOfMathValue = nameOfMathValue;
    }

    public MathMatrixAccessOperatorSymbol getMathMatrixAccessOperatorSymbol() {
        return mathMatrixAccessOperatorSymbol;
    }

    public void setMathMatrixAccessOperatorSymbol(MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol) {
        this.mathMatrixAccessOperatorSymbol = mathMatrixAccessOperatorSymbol;
    }

    public MathAssignmentOperator getAssignmentOperator() {
        return assignmentOperator;
    }

    public void setAssignmentOperator(MathAssignmentOperator assignmentOperator) {
        this.assignmentOperator = assignmentOperator;
    }

    public MathExpressionSymbol getExpressionSymbol() {
        return expressionSymbol;
    }

    public void setExpressionSymbol(MathExpressionSymbol expressionSymbol) {
        this.expressionSymbol = expressionSymbol;
    }

    @Override
    public boolean isAssignmentExpression() {
        return true;
    }

    @Override
    public String getTextualRepresentation() {
        return nameOfMathValue + assignmentOperator.getOperator() + expressionSymbol.getTextualRepresentation() + ";";
    }


    @Override
    public MathExpressionSymbol getAssignedMathExpressionSymbol() {
        return expressionSymbol;
    }
}
