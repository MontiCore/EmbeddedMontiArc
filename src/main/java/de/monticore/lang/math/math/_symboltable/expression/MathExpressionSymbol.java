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

import de.monticore.lang.monticar.interfaces.TextualExpression;
import de.monticore.symboltable.CommonSymbol;

/**
 * All valid MathExpressions
 *
 * @author Sascha Schneiders
 */
public abstract class MathExpressionSymbol extends CommonSymbol implements TextualExpression {
    public static MathExpressionSymbolKind KIND = new MathExpressionSymbolKind();
    protected int id = -1;

    public MathExpressionSymbol() {
        super("", KIND);
    }

    public MathExpressionSymbol(String name) {
        super(name, KIND);
    }

    //Assignments
    public boolean isAssignmentExpression() {
        return false;
    }

    public boolean isAssignmentDeclarationExpression() {
        return false;
    }

    //Comparision
    public boolean isCompareExpression() {
        return false;
    }

    //ForLoop
    public boolean isForLoopExpression() {
        return false;
    }

    //Either IF/ElseIf/Else Expression
    public boolean isConditionalExpression() {
        return false;
    }

    //Whole If/ElseIf/Elseblock
    public boolean isConditionalsExpression() {
        return false;
    }

    public boolean isArithmeticExpression() {
        return false;
    }

    public boolean isValueExpression() {
        return false;
    }

    public boolean isMatrixExpression() {
        return false;
    }

    public boolean isMathValueTypeExpression() {
        return false;
    }

    public boolean isPreOperatorExpression() {
        return false;
    }

    public boolean isParenthesisExpression() {
        return false;
    }

    //returns text
    public abstract String getTextualRepresentation();


    // Can be used if other language need new MathExpressionSymbols in their handling methods,
    // so they do not need to create new symbols
    public int getExpressionID() {
        return id;
    }

    public void setID(int id) {
        this.id = id;
    }

    public MathExpressionSymbol getRealMathExpressionSymbol() {
        return this;
    }

    public MathExpressionSymbol getAssignedMathExpressionSymbol() {
        return this;
    }

    @Override
    public boolean equals(Object object) {
        boolean result = false;
        if (object == null)
            result = false;
        else if (object instanceof MathExpressionSymbol) {
            result = this.getTextualRepresentation().equals(((MathExpressionSymbol) object).getTextualRepresentation());
        }
        return result;
    }
}
