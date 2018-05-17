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
package de.monticore.lang.math._symboltable;

import de.monticore.ast.ASTNode;
import de.monticore.lang.math._symboltable.expression.IArithmeticExpression;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

/**
 * @author Sascha Schneiders
 */
public class MathSymbolTableCreatorHelper {
    public static void setOperatorLeftRightExpression(IArithmeticExpression symbol, ASTNode leftExpressionSymbol, ASTNode rightExpressionSymbol, String operator) {
        if (rightExpressionSymbol != null) {
            setOperatorLeftRightExpression(symbol, (MathExpressionSymbol) leftExpressionSymbol.getSymbolOpt().get(), (MathExpressionSymbol) rightExpressionSymbol.getSymbol().get(), operator);
        } else {
            setOperatorLeftRightExpression(symbol, (MathExpressionSymbol) leftExpressionSymbol.getSymbolOpt().get(), null, operator);
        }
    }

    public static void setOperatorLeftRightExpression(IArithmeticExpression symbol, MathExpressionSymbol leftExpressionSymbol, MathExpressionSymbol rightExpressionSymbol, String operator) {
        symbol.setOperator(operator);
        symbol.setLeftExpression(leftExpressionSymbol);
        symbol.setRightExpression(rightExpressionSymbol);
    }
}
