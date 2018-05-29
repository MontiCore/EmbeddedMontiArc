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

import de.monticore.lang.math._matrixprops.MatrixProperties;

import java.util.ArrayList;

/**
 * Symbol represents a MathValueSymbol which consists of a type and a mathexpression that determines its value.
 *
 * @author Sascha Schneiders
 */
public class MathValueSymbol extends MathValueExpressionSymbol {

    protected MathValueType type;
    protected MathExpressionSymbol value;
    protected ArrayList<MatrixProperties> matrixProperties;

    public MathValueSymbol(String name) {
        super(name);
    }

    public MathValueType getType() {
        return type;
    }

    public void setType(MathValueType type) {
        this.type = type;
    }

    public MathExpressionSymbol getValue() {
        return value;
    }

    public void setValue(MathExpressionSymbol value) {
        this.value = value;
    }

    public ArrayList<MatrixProperties> getMatrixProperties() {
        return matrixProperties;
    }

    public void setMatrixProperties(ArrayList<MatrixProperties> matrixProperties) {
        this.matrixProperties = matrixProperties;
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";
        if (type != null)
            result += type.getTextualRepresentation() + " ";

        result += getFullName();

        if (value != null)
            result += " = " + getValue().getTextualRepresentation();
        return result;
    }

    public boolean isMatrixValueSymbol() {
        return false;
    }

    @Override
    public boolean isAssignmentDeclarationExpression() {
        return true;
    }


    public MathExpressionSymbol getAssignedMathExpressionSymbol() {
        if (value != null)
            return value;
        return this;
    }
}
