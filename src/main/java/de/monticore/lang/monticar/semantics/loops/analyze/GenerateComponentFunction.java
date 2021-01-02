/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.CopyEMAMMathExpressionSymbol;
import de.monticore.lang.math._symboltable.MathAssignmentOperator;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;

import java.util.HashMap;
import java.util.Map;

public class GenerateComponentFunction {

    private static final String tempPostfix = "_TEMP_";

    public static MathStatementsSymbol generateOutput(MathStatementsSymbol executeFunction) {
        MathStatementsSymbol outputFunction = CopyEMAMMathExpressionSymbol.copy(executeFunction);
        Map<String, String> resets = new HashMap<>();
        MathExpressionSymbol last = null;
        if (executeFunction != null) {
            for (MathExpressionSymbol expressionSymbol : outputFunction.getMathExpressionSymbols()) {
                last = expressionSymbol;
                if (expressionSymbol.isAssignmentDeclarationExpression())
                    createCopy(outputFunction, resets, (MathValueSymbol) expressionSymbol);
                else if (expressionSymbol.isValueExpression()) {
                    MathValueExpressionSymbol valueExpressionSymbol = (MathValueExpressionSymbol) expressionSymbol;
                    if (valueExpressionSymbol.isValueExpression())
                        createCopy(outputFunction, resets, (MathValueSymbol) valueExpressionSymbol);
                }
            }
        }
        for (Map.Entry<String, String> reset : resets.entrySet()) {
            MathAssignmentExpressionSymbol resetAssignment = new MathAssignmentExpressionSymbol();
            resetAssignment.setNameOfMathValue(reset.getKey());
            resetAssignment.setAssignmentOperator(new MathAssignmentOperator("="));
            resetAssignment.setExpressionSymbol(new MathNameExpressionSymbol(reset.getValue()));
            outputFunction.addMathExpressionAfter(resetAssignment, last);
            last = resetAssignment;
        }

        return outputFunction;
    }

    public static MathStatementsSymbol generateUpdate(MathStatementsSymbol executeFunction) {
        // Only needs to return the execute function, since update is only called in the end
        return CopyEMAMMathExpressionSymbol.copy(executeFunction);
    }

    private static void createCopy(MathStatementsSymbol outputFunction,
                                   Map<String, String> resets, MathValueSymbol valueExpressionSymbol) {
        MathValueSymbol mathValueSymbol = valueExpressionSymbol;
        if (mathValueSymbol.getType().getProperties().contains("static")) {
            String newName = mathValueSymbol.getName() + tempPostfix;
            resets.put(mathValueSymbol.getName(), newName);

            MathValueSymbol reset = new MathValueSymbol(newName);
            reset.setPackageName(mathValueSymbol.getPackageName());
            reset.setMatrixProperties(mathValueSymbol.getMatrixProperties());
            MathValueType type = CopyEMAMMathExpressionSymbol.copy(mathValueSymbol.getType());
            type.getProperties().remove("static");
            reset.setType(type);

            MathMatrixNameExpressionSymbol copyOld = new MathMatrixNameExpressionSymbol("copy");
            copyOld.setMathMatrixAccessOperatorSymbol(new MathMatrixAccessOperatorSymbol());
            MathMatrixAccessSymbol mathMatrixAccessSymbol =
                    new MathMatrixAccessSymbol(new MathNameExpressionSymbol(mathValueSymbol.getName()));
            copyOld.getMathMatrixAccessOperatorSymbol().addMathMatrixAccessSymbol(mathMatrixAccessSymbol);

            reset.setValue(copyOld);

            outputFunction.addMathExpressionBefore(copyOld, valueExpressionSymbol);
        }
    }

}
