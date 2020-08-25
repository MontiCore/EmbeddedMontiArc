/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math._symboltable.expression.MathConditionalExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.List;

/**
 */
public class ExecuteMethodGeneratorHelper {
    public static String generateIfConditionCode(MathConditionalExpressionSymbol mathConditionalExpressionSymbol, List<String> includeStrings) {

        String result = "";
        //condition
        if (mathConditionalExpressionSymbol.getCondition().isPresent()) {
            result += "if(" + ExecuteMethodGenerator.generateExecuteCode(mathConditionalExpressionSymbol.getCondition().get(), includeStrings) + ")";
        }
        //body
        result += "{\n";
        for (MathExpressionSymbol mathExpressionSymbol : mathConditionalExpressionSymbol.getBodyExpressions())
            result += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, includeStrings);
        result += "}\n";

        return result;
    }
}
