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
package de.monticore.lang.math.math._symboltable;

import de.monticore.lang.math.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
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
