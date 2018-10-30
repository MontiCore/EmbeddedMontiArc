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

import de.monticore.lang.math._symboltable.expression.MathConditionalExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.List;

/**
 * @author Sascha Schneiders
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
