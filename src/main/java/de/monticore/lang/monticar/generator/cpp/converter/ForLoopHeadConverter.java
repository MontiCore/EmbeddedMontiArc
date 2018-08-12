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
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 * @author Christoph Richter
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
