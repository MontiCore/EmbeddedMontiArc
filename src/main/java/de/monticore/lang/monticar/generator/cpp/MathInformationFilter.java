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
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;

import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class MathInformationFilter {
    public static void filterStaticInformation(ExpandedComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint, MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP, List<String> includeStrings) {
        if (mathStatementsSymbol != null) {
            for (MathExpressionSymbol expressionSymbol : mathStatementsSymbol.getMathExpressionSymbols()) {
                if (expressionSymbol.isAssignmentDeclarationExpression()) {
                    MathValueSymbol mathValueSymbol = (MathValueSymbol) expressionSymbol;
                    if (mathValueSymbol.getType().getProperties().contains("static")) {
                        VariableStatic var = new VariableStatic(mathValueSymbol.getName(), Variable.STATIC);
                        var.setTypeNameTargetLanguage(TypeConverter.getVariableTypeNameForMathLanguageTypeName(mathValueSymbol.getType()));
                        if (mathValueSymbol.getValue() != null)
                            var.setAssignmentSymbol(mathValueSymbol.getValue());
                        for (MathExpressionSymbol dimension : mathValueSymbol.getType().getDimensions())
                            var.addDimensionalInformation(dimension.getTextualRepresentation());
                        bluePrint.getMathInformationRegister().addVariable(var);
                    }
                } else if (expressionSymbol.isValueExpression()) {
                    MathValueExpressionSymbol valueExpressionSymbol = (MathValueExpressionSymbol) expressionSymbol;
                    if (valueExpressionSymbol.isValueExpression()) {
                        MathValueSymbol mathValueSymbol = (MathValueSymbol) valueExpressionSymbol;
                        if (mathValueSymbol.getType().getProperties().contains("static")) {
                            VariableStatic var = new VariableStatic(mathValueSymbol.getName(), Variable.STATIC);
                            var.setTypeNameTargetLanguage(TypeConverter.getVariableTypeNameForMathLanguageTypeName(mathValueSymbol.getType()));
                            if (mathValueSymbol.getValue() != null)
                                var.setAssignmentSymbol(mathValueSymbol.getValue());
                            for (MathExpressionSymbol dimension : mathValueSymbol.getType().getDimensions())
                                var.addDimensionalInformation(dimension.getTextualRepresentation());

                            bluePrint.getMathInformationRegister().addVariable(var);
                        }
                    }
                }
            }
        }
    }
}
