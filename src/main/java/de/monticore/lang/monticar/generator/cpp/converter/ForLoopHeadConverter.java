package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.monticore.lang.monticar.generator.Variable;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class ForLoopHeadConverter {
    public static String getForLoopHeadCode(MathForLoopHeadSymbol mathForLoopHeadSymbol, List<String> includeStrings) {
        String result = "";
        result += "for( auto " + mathForLoopHeadSymbol.getNameLoopVariable() + "=";
        if (mathForLoopHeadSymbol.getMathExpression().isMatrixExpression()) {
            MathMatrixExpressionSymbol mathMatrixExpressionSymbol = (MathMatrixExpressionSymbol) mathForLoopHeadSymbol.getMathExpression();
            Log.info(mathMatrixExpressionSymbol.getAstNode().get().toString(), "SHOW ME:");
            if (mathMatrixExpressionSymbol.isMatrixVectorExpression()) {
                MathMatrixVectorExpressionSymbol mathMatrixVectorExpressionSymbol = (MathMatrixVectorExpressionSymbol) mathMatrixExpressionSymbol;

                result += ExecuteMethodGenerator.generateExecuteCode(mathMatrixVectorExpressionSymbol.getStart(), includeStrings) + ";";

                //for loop condition
                result += mathForLoopHeadSymbol.getNameLoopVariable() + "<=" + ExecuteMethodGenerator.generateExecuteCode(mathMatrixVectorExpressionSymbol.getEnd(), includeStrings) + ";";


                //for loop step
                if (mathMatrixVectorExpressionSymbol.getStep().isPresent())
                    result += mathForLoopHeadSymbol.getNameLoopVariable() + "+=" + ExecuteMethodGenerator.generateExecuteCode(mathMatrixVectorExpressionSymbol.getStep().get(), includeStrings) + ")";
                    //increase by one if no step is present
                else
                    result += "++" + mathForLoopHeadSymbol.getNameLoopVariable() + ")";
            }
        }
        return result;
    }
}
