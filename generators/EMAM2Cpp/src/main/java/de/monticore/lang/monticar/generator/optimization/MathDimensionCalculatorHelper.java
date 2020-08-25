/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.math._symboltable.expression.IArithmeticExpression;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
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
