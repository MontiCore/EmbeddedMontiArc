/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.CopyEMAMMathExpressionSymbol;
import de.monticore.lang.math._symboltable.MathAssignmentOperator;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.semantics.setup.Delegate;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class GenerateComponentFunction {

    private static final String tempPostfix = "_TEMP_";

    public static MathStatementsSymbol generateOutput(MathStatementsSymbol executeFunction) {
        MathStatementsSymbol outputFunction = Delegate.copyMathExpressionSymbol(executeFunction);
        Map<String, String> resets = new HashMap<>();
        MathExpressionSymbol last = null;
        if (executeFunction != null) {
            for (MathExpressionSymbol expressionSymbol : outputFunction.getMathExpressionSymbols()) {
                last = expressionSymbol;
                Optional<MathValueSymbol> copy = createCopy(expressionSymbol, resets);
                if (copy.isPresent())
                    outputFunction.addMathExpressionBefore(copy.get(), expressionSymbol);
            }
        }
        for (Map.Entry<String, String> reset : resets.entrySet()) {
            MathAssignmentExpressionSymbol resetAssignment = createReset(reset.getValue(), reset.getKey());
            outputFunction.addMathExpressionAfter(resetAssignment, last);
            last = resetAssignment;
        }

        return outputFunction;
    }

    public static MathAssignmentExpressionSymbol createReset(String oldName, String newName) {
        MathAssignmentExpressionSymbol resetAssignment = new MathAssignmentExpressionSymbol();
        resetAssignment.setNameOfMathValue(oldName);
        resetAssignment.setAssignmentOperator(new MathAssignmentOperator("="));
        resetAssignment.setExpressionSymbol(new MathNameExpressionSymbol(newName));
        return resetAssignment;
    }

    public static MathStatementsSymbol generateUpdate(MathStatementsSymbol executeFunction) {
        // Only needs to return the execute function, since update is only called in the end
        return Delegate.copyMathExpressionSymbol(executeFunction);
    }

    public static Optional<MathValueSymbol> createCopy(MathExpressionSymbol expressionSymbol,
                                                       Map<String, String> resets) {
        MathValueSymbol mathValueSymbol = null;
        if (expressionSymbol.isAssignmentDeclarationExpression())
            mathValueSymbol = (MathValueSymbol) expressionSymbol;
        else if (expressionSymbol.isValueExpression()) {
            MathValueExpressionSymbol valueExpressionSymbol = (MathValueExpressionSymbol) expressionSymbol;
            if (valueExpressionSymbol.isValueExpression())
                mathValueSymbol = (MathValueSymbol) valueExpressionSymbol;
        }

        if (mathValueSymbol == null) return Optional.empty();

        if (mathValueSymbol.getType().getProperties().contains("static")) {
            String newName = mathValueSymbol.getName() + tempPostfix;
            resets.put(newName, mathValueSymbol.getName());

            MathValueSymbol reset = new MathValueSymbol(newName);
            reset.setPackageName(mathValueSymbol.getPackageName());
            reset.setMatrixProperties(mathValueSymbol.getMatrixProperties());
            MathValueType type = Delegate.copyMathExpressionSymbol(mathValueSymbol.getType());
            type.getProperties().remove("static");
            reset.setType(type);

            MathMatrixNameExpressionSymbol copyOld = new MathMatrixNameExpressionSymbol("copy");
            copyOld.setMathMatrixAccessOperatorSymbol(new MathMatrixAccessOperatorSymbol());
            MathMatrixAccessSymbol mathMatrixAccessSymbol =
                    new MathMatrixAccessSymbol(new MathNameExpressionSymbol(mathValueSymbol.getName()));
            copyOld.getMathMatrixAccessOperatorSymbol().addMathMatrixAccessSymbol(mathMatrixAccessSymbol);

            reset.setValue(copyOld);

            return Optional.of(reset);
        }
        return Optional.empty();
    }

}
