/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class MathExpressionReplacer {

    public static void replaceMathExpression(MathStatementsSymbol mathStatementsSymbol, MathExpressionSymbol newMathExpression, MathExpressionSymbol oldMathExpressionSymbol) {
        int id = 0;
        List<Integer> removeIds = new ArrayList<>();
        for (MathExpressionSymbol mathExpressionSymbol : mathStatementsSymbol.getMathExpressionSymbols()) {
            if (shouldReplaceExpression(mathExpressionSymbol, oldMathExpressionSymbol)) {
                removeIds.add(id);
            } else
                replace(mathExpressionSymbol, newMathExpression, oldMathExpressionSymbol);
            ++id;
        }
        for (Integer removeId : removeIds) {
            mathStatementsSymbol.getMathExpressionSymbols().add(removeId, newMathExpression);
            mathStatementsSymbol.getMathExpressionSymbols().remove(removeId + 1);
        }
    }

    private static void replace(MathExpressionSymbol currentMathExpressionSymbol, MathExpressionSymbol newMathExpressionSymbol, MathExpressionSymbol oldMathExpressionSymbol) {
        if (currentMathExpressionSymbol.isAssignmentExpression()) {
            replace((MathAssignmentExpressionSymbol) currentMathExpressionSymbol, newMathExpressionSymbol, oldMathExpressionSymbol);
        } else if (currentMathExpressionSymbol.isArithmeticExpression()) {
            replace((MathArithmeticExpressionSymbol) currentMathExpressionSymbol, newMathExpressionSymbol, oldMathExpressionSymbol);
        } else if (currentMathExpressionSymbol.isMatrixExpression()) {
            replace((MathMatrixExpressionSymbol) currentMathExpressionSymbol, newMathExpressionSymbol, oldMathExpressionSymbol);
        } else {
            logErrorExpression(currentMathExpressionSymbol);
        }
    }

    private static void replace(MathAssignmentExpressionSymbol currentMathExpressionSymbol, MathExpressionSymbol newMathExpressionSymbol, MathExpressionSymbol oldMathExpressionSymbol) {
        if (shouldReplaceExpression(currentMathExpressionSymbol.getExpressionSymbol(), oldMathExpressionSymbol))
            currentMathExpressionSymbol.setExpressionSymbol(newMathExpressionSymbol);
        else
            replace(currentMathExpressionSymbol.getExpressionSymbol(), newMathExpressionSymbol, oldMathExpressionSymbol);
    }

    private static void replace(MathArithmeticExpressionSymbol currentMathExpressionSymbol, MathExpressionSymbol newMathExpressionSymbol, MathExpressionSymbol oldMathExpressionSymbol) {
        if (shouldReplaceExpression(currentMathExpressionSymbol.getLeftExpression(), oldMathExpressionSymbol))
            currentMathExpressionSymbol.setLeftExpression(newMathExpressionSymbol);
        else
            replace(currentMathExpressionSymbol.getLeftExpression(), newMathExpressionSymbol, oldMathExpressionSymbol);
        if (shouldReplaceExpression(currentMathExpressionSymbol.getRightExpression(), oldMathExpressionSymbol))
            currentMathExpressionSymbol.setRightExpression(newMathExpressionSymbol);
        else
            replace(currentMathExpressionSymbol.getRightExpression(), newMathExpressionSymbol, oldMathExpressionSymbol);
    }

    private static void replace(MathMatrixExpressionSymbol currentMathExpressionSymbol, MathExpressionSymbol newMathExpressionSymbol, MathExpressionSymbol oldMathExpressionSymbol) {
        logErrorExpression(currentMathExpressionSymbol);
    }

    private static void replace(MathMatrixAccessOperatorSymbol currentMathExpressionSymbol, MathExpressionSymbol newMathExpressionSymbol, MathExpressionSymbol oldMathExpressionSymbol) {
        logErrorExpression(currentMathExpressionSymbol);
    }

    private static void logErrorExpression(MathExpressionSymbol currentMathExpressionSymbol) {
        Log.debug("Not handled replaceMathExpression: " + currentMathExpressionSymbol.getClass().getName() + " " + currentMathExpressionSymbol.getTextualRepresentation(), "MathExpressionReplacer");
    }

    private static boolean shouldReplaceExpression(MathExpressionSymbol encounteredExpressionSymbol, MathExpressionSymbol oldMathExpressionSymbol) {
        return encounteredExpressionSymbol.equals(oldMathExpressionSymbol);
    }
}
