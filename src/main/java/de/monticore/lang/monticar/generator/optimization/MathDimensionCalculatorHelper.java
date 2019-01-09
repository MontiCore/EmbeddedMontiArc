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
package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.math._symboltable.expression.IArithmeticExpression;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class MathDimensionCalculatorHelper {
    public static int calculateMatrixColumns(IArithmeticExpression mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        int result = 0;
        if (mathExpressionSymbol.getOperator().equals("+")) {
            MathExpressionSymbol realLeftExpression = MathOptimizer.getCurrentAssignment(mathExpressionSymbol.getLeftExpression(), precedingExpressions);
            MathExpressionSymbol realRightExpression = MathOptimizer.getCurrentAssignment(mathExpressionSymbol.getRightExpression(), precedingExpressions);
            result = MathDimensionCalculatorHelper.calculateMatrixColumns(realLeftExpression, realRightExpression, precedingExpressions);
        } else if (mathExpressionSymbol.getOperator().equals("*")) {
            MathExpressionSymbol realLeftExpression = MathOptimizer.getCurrentAssignment(mathExpressionSymbol.getLeftExpression(), precedingExpressions);
            MathExpressionSymbol realRightExpression = MathOptimizer.getCurrentAssignment(mathExpressionSymbol.getRightExpression(), precedingExpressions);
            result = MathDimensionCalculatorHelper.calculateMatrixColumns(realLeftExpression, realRightExpression, precedingExpressions);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName(), "Not handled:");
        }
        return result;
    }

    public static int calculateMatrixColumns(MathExpressionSymbol realLeftExpression, MathExpressionSymbol realRightExpression, List<MathExpressionSymbol> precedingExpressions) {
        int result = 0;
        if (MathDimensionCalculator.getMatrixRows(realRightExpression, precedingExpressions) == 1) {
            result = MathDimensionCalculator.getMatrixColumns(realLeftExpression, precedingExpressions);
        } else {
            result = MathDimensionCalculator.getMatrixColumns(realLeftExpression, precedingExpressions);
        }
        return result;
    }

    public static int calculateMatrixRows(IArithmeticExpression mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        int result = 0;
        if (mathExpressionSymbol.getOperator().equals("+")) {
            MathExpressionSymbol realLeftExpression = MathOptimizer.getCurrentAssignment(mathExpressionSymbol.getLeftExpression(), precedingExpressions);
            MathExpressionSymbol realRightExpression = MathOptimizer.getCurrentAssignment(mathExpressionSymbol.getRightExpression(), precedingExpressions);
            result = calculateMatrixRows(realLeftExpression, realRightExpression, precedingExpressions);
        } else if (mathExpressionSymbol.getOperator().equals("*")) {
            MathExpressionSymbol realLeftExpression = MathOptimizer.getCurrentAssignment(mathExpressionSymbol.getLeftExpression(), precedingExpressions);
            MathExpressionSymbol realRightExpression = MathOptimizer.getCurrentAssignment(mathExpressionSymbol.getRightExpression(), precedingExpressions);
            result = calculateMatrixRows(realLeftExpression, realRightExpression, precedingExpressions);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName(), "Not handled:");
        }
        return result;
    }

    public static int calculateMatrixRows(MathExpressionSymbol realLeftExpression, MathExpressionSymbol realRightExpression, List<MathExpressionSymbol> precedingExpressions) {
        int result;
        if (MathDimensionCalculator.getMatrixColumns(realRightExpression, precedingExpressions) == 1) {
            result = MathDimensionCalculator.getMatrixRows(realLeftExpression, precedingExpressions);
        } else {
            result = MathDimensionCalculator.getMatrixRows(realRightExpression, precedingExpressions);
        }
        return result;
    }
}
