/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 */
public class MathDimensionCalculator {

    public static int getMatrixColumns(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        int result = 0;
        if (mathExpressionSymbol.isMatrixExpression()) {
            result = getMatrixColumns((MathMatrixExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isArithmeticExpression()) {
            result = getMatrixColumns((MathArithmeticExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isValueExpression()) {
            result = getMatrixColumns((MathValueExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isParenthesisExpression()) {
            result = getMatrixColumns((MathParenthesisExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isPreOperatorExpression()) {
            result = getMatrixColumns(((MathPreOperatorExpressionSymbol) mathExpressionSymbol).getMathExpressionSymbol(), precedingExpressions);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName(), "getMatrixColumns1 Not handled:");
        }
        return result;
    }

    public static int getMatrixColumns(MathMatrixExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        int result = 0;
        if (mathExpressionSymbol.isMatrixNameExpression()) {
            result = getMatrixColumns((MathMatrixNameExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isMatrixArithmeticExpression()) {
            result = getMatrixColumns((MathMatrixArithmeticExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName(), "getMatrixColumns2 Not handled:");
        }
        return result;
    }


    public static int getMatrixColumns(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        return MathDimensionCalculatorHelper.calculateMatrixColumns(mathExpressionSymbol, precedingExpressions);
    }


    public static int getMatrixColumns(MathArithmeticExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        return MathDimensionCalculatorHelper.calculateMatrixColumns(mathExpressionSymbol, precedingExpressions);
    }

    public static int getMatrixColumns(MathMatrixNameExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        MathExpressionSymbol currentAssignment = MathOptimizer.getCurrentAssignment(mathExpressionSymbol, precedingExpressions);
        if (mathExpressionSymbol.equals(currentAssignment)) {
            return MathOptimizer.currentBluePrint.getMathInformationRegister().getAmountColumns(mathExpressionSymbol.getNameToAccess(), mathExpressionSymbol.getMathMatrixAccessOperatorSymbol());
        }
        return getMatrixColumns(currentAssignment, precedingExpressions);
    }

    public static int getMatrixColumns(MathValueExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        int result = 0;
        if (mathExpressionSymbol.isNameExpression()) {
            result = getMatrixColumns((MathNameExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isNumberExpression()) {
            result = 1;
        } else if (mathExpressionSymbol.isValueExpression()) {
            result = getMatrixColumns((MathValueSymbol) mathExpressionSymbol, precedingExpressions);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName(), "getMatrixColumns3 Not handled:");
        }
        return result;
    }

    public static int getMatrixColumns(MathNameExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        MathExpressionSymbol currentExpression = MathOptimizer.getCurrentAssignment(mathExpressionSymbol, precedingExpressions);
        if (currentExpression.equals(mathExpressionSymbol)) {
            return MathOptimizer.currentBluePrint.getMathInformationRegister().getAmountColumns(mathExpressionSymbol.getNameToResolveValue());
        } else {
            return getMatrixColumns(currentExpression, precedingExpressions);
        }
    }

    public static int getMatrixColumns(MathValueSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        return getMatrixColumns(mathExpressionSymbol.getValue(), precedingExpressions);
    }

    public static int getMatrixColumns(MathParenthesisExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        return getMatrixColumns(mathExpressionSymbol.getMathExpressionSymbol(), precedingExpressions);
    }

    public static int getMatrixRows(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        int result = 0;
        if (mathExpressionSymbol.isMatrixExpression()) {
            result = getMatrixRows((MathMatrixExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isArithmeticExpression()) {
            result = getMatrixRows((MathArithmeticExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isValueExpression()) {
            result = getMatrixRows((MathValueExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isParenthesisExpression()) {
            result = getMatrixRows((MathParenthesisExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isPreOperatorExpression()) {
            result = getMatrixRows(((MathPreOperatorExpressionSymbol) mathExpressionSymbol).getMathExpressionSymbol(), precedingExpressions);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName(), "getMatrixRows1 Not handled:");
        }
        return result;
    }

    public static int getMatrixRows(MathMatrixExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        int result = 0;
        if (mathExpressionSymbol.isMatrixNameExpression()) {
            result = getMatrixRows((MathMatrixNameExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isMatrixArithmeticExpression()) {
            result = getMatrixRows((MathMatrixArithmeticExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName(), "getMatrixRows2 Not handled:");
        }
        return result;
    }


    public static int getMatrixRows(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        return MathDimensionCalculatorHelper.calculateMatrixRows(mathExpressionSymbol, precedingExpressions);
    }

    public static int getMatrixRows(MathArithmeticExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        return MathDimensionCalculatorHelper.calculateMatrixRows(mathExpressionSymbol, precedingExpressions);
    }

    public static int getMatrixRows(MathMatrixNameExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        MathExpressionSymbol currentAssignment = MathOptimizer.getCurrentAssignment(mathExpressionSymbol, precedingExpressions);
        if (mathExpressionSymbol.equals(currentAssignment)) {
            return MathOptimizer.currentBluePrint.getMathInformationRegister().getAmountRows(mathExpressionSymbol.getNameToAccess(), mathExpressionSymbol.getMathMatrixAccessOperatorSymbol());
        }
        return getMatrixRows(currentAssignment, precedingExpressions);
    }

    public static int getMatrixRows(MathValueExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        int result = 0;
        if (mathExpressionSymbol.isNameExpression()) {
            result = getMatrixRows((MathNameExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isNumberExpression()) {
            result = 1;
        } else if (mathExpressionSymbol.isValueExpression()) {
            result = getMatrixRows((MathValueSymbol) mathExpressionSymbol, precedingExpressions);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName(), "getMatrixRows3 Not handled:");
        }
        return result;
    }

    public static int getMatrixRows(MathNameExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        MathExpressionSymbol currentExpression = MathOptimizer.getCurrentAssignment(mathExpressionSymbol, precedingExpressions);
        if (currentExpression.equals(mathExpressionSymbol)) {
            return MathOptimizer.currentBluePrint.getMathInformationRegister().getAmountRows(mathExpressionSymbol.getNameToResolveValue());
        } else {
            return getMatrixRows(currentExpression, precedingExpressions);
        }
    }

    public static int getMatrixRows(MathValueSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        return getMatrixRows(mathExpressionSymbol.getValue(), precedingExpressions);
    }

    public static int getMatrixRows(MathParenthesisExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        return getMatrixRows(mathExpressionSymbol.getMathExpressionSymbol(), precedingExpressions);
    }
}
