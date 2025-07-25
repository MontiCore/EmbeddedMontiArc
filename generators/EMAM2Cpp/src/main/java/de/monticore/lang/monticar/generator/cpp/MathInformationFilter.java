/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;

import java.util.List;

/**
 */
public class MathInformationFilter {
    public static void filterStaticInformation(EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint, MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP, List<String> includeStrings) {
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
