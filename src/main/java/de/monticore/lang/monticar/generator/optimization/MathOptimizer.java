package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.math.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class MathOptimizer {
    public static BluePrintCPP currentBluePrint;
    public static List<MathOptimizationRule> optimizationRules = new ArrayList<>();

    public static void addOptimizationRule(MathOptimizationRule mathOptimizationRule) {
        optimizationRules.add(mathOptimizationRule);
    }

    public static MathExpressionSymbol applyOptimizations(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions, MathStatementsSymbol mathStatementsSymbol) {
        MathExpressionSymbol currentMathExpressionSymbol = mathExpressionSymbol;
        if (mathStatementsSymbol == null) {
            Log.error("MathStatementsSymbol is null but should not be at this point!");
        }
        MathExpressionSymbol lastMathExpressionSymbol = null;
        for (MathOptimizationRule mathOptimizationRule : optimizationRules) {
            mathOptimizationRule.optimize(currentMathExpressionSymbol, precedingExpressions, mathStatementsSymbol);
        }
        return currentMathExpressionSymbol;
    }

    public static MathExpressionSymbol getCurrentAssignment(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressionSymbols) {
        if (mathExpressionSymbol == null)
            return mathExpressionSymbol;
        if (mathExpressionSymbol.isValueExpression()) {
            if (((MathValueExpressionSymbol) mathExpressionSymbol).isNameExpression()) {
                return getCurrentAssignment((MathNameExpressionSymbol) mathExpressionSymbol, precedingExpressionSymbols);
            }
        } else if (mathExpressionSymbol.isMatrixExpression()) {
            if (((MathMatrixExpressionSymbol) mathExpressionSymbol).isMatrixNameExpression()) {
                return MathOptimizer.getCurrentAssignment((MathMatrixNameExpressionSymbol) mathExpressionSymbol, precedingExpressionSymbols);
            }
        } else if (mathExpressionSymbol.isParenthesisExpression()) {
            return getCurrentAssignment(((MathParenthesisExpressionSymbol) mathExpressionSymbol).getMathExpressionSymbol(), precedingExpressionSymbols);
        }

        return mathExpressionSymbol;
    }

    public static MathExpressionSymbol getCurrentAssignment(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, List<MathExpressionSymbol> precedingExpressionSymbols) {
        MathExpressionSymbol currentAssignment = mathMatrixNameExpressionSymbol;

        for (MathExpressionSymbol expressionSymbol : precedingExpressionSymbols) {
            if (expressionSymbol.isAssignmentDeclarationExpression()) {
                MathValueSymbol mathValueSymbol = (MathValueSymbol) expressionSymbol;
                if (mathValueSymbol.getName().equals(mathMatrixNameExpressionSymbol.getNameToAccess())) {
                    currentAssignment = getCurrentAssignment(mathValueSymbol.getValue(), precedingExpressionSymbols);
                }
            } else if (expressionSymbol.isAssignmentExpression()) {
                MathAssignmentExpressionSymbol mathAssignmentExpressionSymbol = (MathAssignmentExpressionSymbol) expressionSymbol;
                if (mathAssignmentExpressionSymbol.getAssignmentOperator().getOperator().equals("=") && mathAssignmentExpressionSymbol.getNameOfMathValue().equals(mathMatrixNameExpressionSymbol.getNameToAccess())) {
                    currentAssignment = getCurrentAssignment(mathAssignmentExpressionSymbol.getExpressionSymbol(), precedingExpressionSymbols);
                }
            }
        }
        return currentAssignment;
    }

    public static MathExpressionSymbol getCurrentAssignment(MathNameExpressionSymbol mathNameExpressionSymbol, List<MathExpressionSymbol> precedingExpressionSymbols) {
        MathExpressionSymbol currentAssignment = mathNameExpressionSymbol;

        for (MathExpressionSymbol expressionSymbol : precedingExpressionSymbols) {
            if(expressionSymbol==null){

            }
            else if (expressionSymbol.isAssignmentDeclarationExpression()) {
                MathValueSymbol mathValueSymbol = (MathValueSymbol) expressionSymbol;
                if (mathValueSymbol.getName().equals(mathNameExpressionSymbol.getNameToResolveValue())) {
                    currentAssignment = getCurrentAssignment(mathValueSymbol.getValue(), precedingExpressionSymbols);
                }
            } else if (expressionSymbol.isAssignmentExpression()) {
                MathAssignmentExpressionSymbol mathAssignmentExpressionSymbol = (MathAssignmentExpressionSymbol) expressionSymbol;
                if (mathAssignmentExpressionSymbol.getAssignmentOperator().getOperator().equals("=") && mathAssignmentExpressionSymbol.getNameOfMathValue().equals(mathNameExpressionSymbol.getNameToResolveValue())) {
                    currentAssignment = getCurrentAssignment(mathAssignmentExpressionSymbol.getExpressionSymbol(), precedingExpressionSymbols);
                }
            }
        }
        return currentAssignment;
    }


    public static int getEstimatedOperations(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        int returnInt;
        if (mathExpressionSymbol.isArithmeticExpression()) {
            returnInt = getEstimatedOperations((MathArithmeticExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isMatrixExpression()) {
            returnInt = getEstimatedOperations((MathMatrixArithmeticExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isParenthesisExpression()) {
            returnInt = getEstimatedOperations(((MathParenthesisExpressionSymbol) mathExpressionSymbol).getMathExpressionSymbol(), precedingExpressions);
        } else {
            returnInt = -1;
        }
        return returnInt;
    }

    public static int getEstimatedOperations(MathArithmeticExpressionSymbol mathArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        int result = 0;
        if (mathArithmeticExpressionSymbol.getMathOperator().equals("*") && mathArithmeticExpressionSymbol.getLeftExpression() != null && mathArithmeticExpressionSymbol.getRightExpression() != null) {
            result = handleEstimatedOperationsMultiplication(mathArithmeticExpressionSymbol, precedingExpressions);
        } else if (mathArithmeticExpressionSymbol.getMathOperator().equals("+")) {
            result = handleEstimatedOperationsAddition(mathArithmeticExpressionSymbol, precedingExpressions);
        }
        Log.info(mathArithmeticExpressionSymbol.getClass().getName() + " operator: " + mathArithmeticExpressionSymbol.getMathOperator(), "Not handled:");
        return result;
    }

    public static int handleEstimatedOperationsAddition(MathArithmeticExpressionSymbol mathArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        MathExpressionSymbol realLeftExpressionSymbol = getCurrentAssignment(mathArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions);
        MathExpressionSymbol realRightExpressionSymbol = getCurrentAssignment(mathArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), precedingExpressions);
        int previousOperations = calculatePreviousOperations(realLeftExpressionSymbol, realRightExpressionSymbol, precedingExpressions);

        return previousOperations + MathDimensionCalculator.getMatrixColumns(realLeftExpressionSymbol, precedingExpressions) * MathDimensionCalculator.getMatrixRows(realRightExpressionSymbol, precedingExpressions);

    }

    public static int handleEstimatedOperationsMultiplication(MathArithmeticExpressionSymbol mathArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        MathExpressionSymbol realLeftExpressionSymbol = getCurrentAssignment(mathArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions);
        MathExpressionSymbol realRightExpressionSymbol = getCurrentAssignment(mathArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), precedingExpressions);
        int n = MathDimensionCalculator.getMatrixColumns(realLeftExpressionSymbol, precedingExpressions);
        int m = MathDimensionCalculator.getMatrixRows(realLeftExpressionSymbol, precedingExpressions);
        int p = MathDimensionCalculator.getMatrixRows(realRightExpressionSymbol, precedingExpressions);
        int previousOperations = 0;
        previousOperations = calculatePreviousOperations(realLeftExpressionSymbol, realRightExpressionSymbol, precedingExpressions);
        return previousOperations + n * m * p;
    }

    public static int calculatePreviousOperations(MathExpressionSymbol realLeftExpressionSymbol, MathExpressionSymbol realRightExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        int previousOperations = 0;
        if (isArithmeticExpression(realLeftExpressionSymbol, "*", precedingExpressions)) {
            previousOperations += getEstimatedOperations(realLeftExpressionSymbol, precedingExpressions);
        } else if (isArithmeticExpression(realLeftExpressionSymbol, "+", precedingExpressions)) {
            previousOperations += getEstimatedOperations(realLeftExpressionSymbol, precedingExpressions);
        }
        if (isArithmeticExpression(realRightExpressionSymbol, "*", precedingExpressions)) {
            previousOperations += getEstimatedOperations(realRightExpressionSymbol, precedingExpressions);
        } else if (isArithmeticExpression(realRightExpressionSymbol, "+", precedingExpressions)) {
            previousOperations += getEstimatedOperations(realRightExpressionSymbol, precedingExpressions);
        }
        return previousOperations;
    }

    public static int getEstimatedOperations(MathMatrixArithmeticExpressionSymbol mathArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (mathArithmeticExpressionSymbol.getMathOperator().equals("*")) {
            MathExpressionSymbol realLeftExpressionSymbol = getCurrentAssignment(mathArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions);
            MathExpressionSymbol realRightExpressionSymbol = getCurrentAssignment(mathArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), precedingExpressions);

            return MathDimensionCalculator.getMatrixColumns(realLeftExpressionSymbol, precedingExpressions) * MathDimensionCalculator.getMatrixRows(realLeftExpressionSymbol, precedingExpressions) * MathDimensionCalculator.getMatrixRows(realRightExpressionSymbol, precedingExpressions);

        } else if (mathArithmeticExpressionSymbol.getMathOperator().equals("+")) {
            MathExpressionSymbol realLeftExpressionSymbol = getCurrentAssignment(mathArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions);
            MathExpressionSymbol realRightExpressionSymbol = getCurrentAssignment(mathArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), precedingExpressions);

            return MathDimensionCalculator.getMatrixColumns(realLeftExpressionSymbol, precedingExpressions) * MathDimensionCalculator.getMatrixRows(realRightExpressionSymbol, precedingExpressions);
        }
        Log.info(mathArithmeticExpressionSymbol.getClass().getName() + " operator: " + mathArithmeticExpressionSymbol.getMathOperator(), "Not handled:");
        return 0;
    }

    /**
     * Returns true if the expression is any kind of arithmetic expressions which has the provided operator as operator.
     */
    public static boolean isArithmeticExpression(MathExpressionSymbol mathExpressionSymbol, String operator, List<MathExpressionSymbol> precedingExpressionSymbols) {
        MathExpressionSymbol currentMathExpressionSymbol = mathExpressionSymbol;
        if (currentMathExpressionSymbol.isValueExpression()) {
            MathValueExpressionSymbol mathValueExpressionSymbol = (MathValueExpressionSymbol) currentMathExpressionSymbol;
            if (mathValueExpressionSymbol.isNameExpression()) {
                currentMathExpressionSymbol = MathOptimizer.getCurrentAssignment((MathNameExpressionSymbol) mathValueExpressionSymbol, precedingExpressionSymbols);
            }
        }
        if (currentMathExpressionSymbol.isArithmeticExpression()) {

            return ((MathArithmeticExpressionSymbol) currentMathExpressionSymbol).getMathOperator().equals(operator);
        } else if (currentMathExpressionSymbol.isMatrixExpression()) {
            MathMatrixExpressionSymbol mathMatrixExpressionSymbol = (MathMatrixExpressionSymbol) currentMathExpressionSymbol;
            if (mathMatrixExpressionSymbol.isMatrixArithmeticExpression())
                return ((MathMatrixArithmeticExpressionSymbol) mathMatrixExpressionSymbol).getMathOperator().equals(operator);
        }
        return false;
    }

    public static boolean isArithmeticMatrixExpression(MathExpressionSymbol mathExpressionSymbol, String operator, List<MathExpressionSymbol> precedingExpressionSymbols) {
        if (mathExpressionSymbol.isMatrixExpression()) {
            MathMatrixExpressionSymbol mathMatrixExpressionSymbol = (MathMatrixExpressionSymbol) mathExpressionSymbol;
            if (mathMatrixExpressionSymbol.isMatrixArithmeticExpression())
                return ((MathMatrixArithmeticExpressionSymbol) mathMatrixExpressionSymbol).getMathOperator().equals(operator);
        }
        return false;
    }

    public static MathExpressionSymbol getLeftExpressionSymbol(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        Log.info(mathExpressionSymbol.getClass().getName() + " " + mathExpressionSymbol.getTextualRepresentation(), "Class of Symbol:");
        if (mathExpressionSymbol.isArithmeticExpression() && ((MathArithmeticExpressionSymbol) mathExpressionSymbol).getLeftExpression() != null) {
            return MathOptimizer.getCurrentAssignment(((MathArithmeticExpressionSymbol) mathExpressionSymbol).getLeftExpression(), precedingExpressions);
        } else if (mathExpressionSymbol.isMatrixExpression()) {
            if (((MathMatrixExpressionSymbol) mathExpressionSymbol).isMatrixArithmeticExpression()) {
                return MathOptimizer.getCurrentAssignment(((MathMatrixArithmeticExpressionSymbol) mathExpressionSymbol).getLeftExpression(), precedingExpressions);
            }
        }

        return null;
    }

    public static MathExpressionSymbol getRightExpressionSymbol(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        Log.info(mathExpressionSymbol.getClass().getName() + " " + mathExpressionSymbol.getTextualRepresentation(), "Class of Symbol:");
        if (mathExpressionSymbol.isArithmeticExpression()) {
            return MathOptimizer.getCurrentAssignment(((MathArithmeticExpressionSymbol) mathExpressionSymbol).getRightExpression(), precedingExpressions);
        } else if (mathExpressionSymbol.isMatrixExpression()) {
            if (((MathMatrixExpressionSymbol) mathExpressionSymbol).isMatrixArithmeticExpression()) {
                return MathOptimizer.getCurrentAssignment(((MathMatrixArithmeticExpressionSymbol) mathExpressionSymbol).getRightExpression(), precedingExpressions);
            }
        }

        return null;
    }

    static {
        addOptimizationRule(new MathMultiplicationAddition());
        addOptimizationRule(new MathMatrixMultiplicationOrder());
        addOptimizationRule(new MathDiagonalMatrixInversionOptimization());
        addOptimizationRule(new MathAssignmentPartResultReuse());
    }


}
