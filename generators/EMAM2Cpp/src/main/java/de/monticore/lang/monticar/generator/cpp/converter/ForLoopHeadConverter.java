/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.Optional;

/**
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

    private static MathMatrixVectorExpressionSymbol getLoopMatrixVectorExpressionSymbol(MathForLoopHeadSymbol mathForLoopHeadSymbol) {
        MathMatrixVectorExpressionSymbol result = null;
        if (mathForLoopHeadSymbol.getMathExpression().isMatrixExpression()) {
            MathMatrixExpressionSymbol mathMatrixExpressionSymbol = (MathMatrixExpressionSymbol) mathForLoopHeadSymbol.getMathExpression();
            if (mathMatrixExpressionSymbol.isMatrixVectorExpression()) {
                result = (MathMatrixVectorExpressionSymbol) mathMatrixExpressionSymbol;
            }
        }
        return result;
    }

    public static Optional<MathExpressionSymbol> getForLoopStart(MathForLoopHeadSymbol mathForLoopHeadSymbol) {
        Optional<MathExpressionSymbol> start = Optional.empty();
        MathMatrixVectorExpressionSymbol loopExpr = getLoopMatrixVectorExpressionSymbol(mathForLoopHeadSymbol);
        if (loopExpr != null)
            start = Optional.ofNullable(loopExpr.getStart());
        return start;
    }

    public static Optional<MathExpressionSymbol> getForLoopEnd(MathForLoopHeadSymbol mathForLoopHeadSymbol) {
        Optional<MathExpressionSymbol> end = Optional.empty();
        MathMatrixVectorExpressionSymbol loopExpr = getLoopMatrixVectorExpressionSymbol(mathForLoopHeadSymbol);
        if (loopExpr != null)
            end = Optional.ofNullable(loopExpr.getEnd());
        return end;
    }

    public static Optional<MathExpressionSymbol> getForLoopStep(MathForLoopHeadSymbol mathForLoopHeadSymbol) {
        Optional<MathExpressionSymbol> end = Optional.empty();
        MathMatrixVectorExpressionSymbol loopExpr = getLoopMatrixVectorExpressionSymbol(mathForLoopHeadSymbol);
        if (loopExpr != null)
            end = loopExpr.getStep();
        return end;
    }
}
