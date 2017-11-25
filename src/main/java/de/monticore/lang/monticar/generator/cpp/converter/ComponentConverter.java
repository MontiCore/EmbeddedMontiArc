package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.lang.math.math._symboltable.*;
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.*;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.*;
import de.monticore.lang.monticar.generator.cpp.instruction.ConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.instruction.ConstantConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.symbols.MathChainedExpression;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;
import de.monticore.lang.monticar.generator.optimization.MathInformationRegister;
import de.monticore.lang.monticar.generator.optimization.MathOptimizer;
import de.monticore.lang.monticar.mcexpressions._ast.ASTExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 *         Handles code generation for a component and its "subsymbols"
 */
public class ComponentConverter {

    public static BluePrintCPP currentBluePrint = null;

    public static BluePrint convertComponentSymbolToBluePrint(ExpandedComponentInstanceSymbol componentSymbol, MathStatementsSymbol mathStatementsSymbol, List<String> includeStrings, GeneratorCPP generatorCPP) {
        BluePrintCPP bluePrint = new BluePrintCPP(GeneralHelperMethods.getTargetLanguageComponentName(componentSymbol.getFullName()));
        ComponentConverter.currentBluePrint = bluePrint;
        bluePrint.setGenerator(generatorCPP);
        bluePrint.setOriginalSymbol(componentSymbol);
        bluePrint.addDefineGenerics(componentSymbol);

        addVariables(componentSymbol, bluePrint);

        String lastNameWithoutArrayPart = "";
        for (ExpandedComponentInstanceSymbol instanceSymbol : componentSymbol.getSubComponents()) {
            int arrayBracketIndex = instanceSymbol.getName().indexOf("[");
            boolean generateComponentInstance = true;
            if (arrayBracketIndex != -1) {
                generateComponentInstance = !instanceSymbol.getName().substring(0, arrayBracketIndex).equals(lastNameWithoutArrayPart);
                lastNameWithoutArrayPart = instanceSymbol.getName().substring(0, arrayBracketIndex);
                Log.info(lastNameWithoutArrayPart, "Without:");
                Log.info(generateComponentInstance + "", "Bool:");
            }
            if (generateComponentInstance) {
            }
            bluePrint.addVariable(ComponentInstanceConverter.convertComponentInstanceSymbolToVariable(instanceSymbol, componentSymbol));
        }

        //create arrays from variables that only differ at the end by _number_
        BluePrintFixer.fixBluePrintVariableArrays(bluePrint);
        MathInformationFilter.filterStaticInformation(componentSymbol, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);
        generateInitMethod(componentSymbol, bluePrint, generatorCPP, includeStrings);

        //generate execute method
        ComponentConverterMethodGeneration.generateExecuteMethod(componentSymbol, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);


        return bluePrint;
    }

    public static void addVariables(ExpandedComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint) {
        //add parameters as variables
        for (EMAVariable variable : componentSymbol.getParameters()) {
            System.out.println("EMAVAR: " + variable.getName() + " " + variable.getType().toString());
            Variable var = new Variable();
            var.setName(variable.getName());
            var.setTypeNameMontiCar(variable.getType());
            bluePrint.addVariable(var);
            bluePrint.getMathInformationRegister().addVariable(var);
            var.setIsParameterVariable(true);
        }
        //add ports as variables to blueprint
        for (PortSymbol port : componentSymbol.getPorts()) {
            bluePrint.addVariable(PortConverter.convertPortSymbolToVariable(port, port.getName(), bluePrint));
        }
    }

    public static void generateInitMethod(ExpandedComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings) {
        Method method = new Method("init", "void");
        bluePrint.addMethod(method);
        for (Variable v : bluePrint.getMathInformationRegister().getVariables()) {
            if (v.isStaticVariable()) {
                generateInitStaticVariablePart(method, v, bluePrint);
            } else {
                generateInitNonStaticVariable(method, v, bluePrint);
            }
        }
        for (Variable v : bluePrint.getVariables()) {
            Log.info("Variable: " + v.getName(), "initBluePrintCreate:");

            if (v.isInputVariable() && !v.isConstantVariable()) {
                //method.addParameter(v);
                //Instruction instruction = new ConnectInstructionCPP(v, true, v, false);
                //method.addInstruction(instruction);
            } else if (v.isConstantVariable()) {
                Instruction instruction = new ConstantConnectInstructionCPP(v, v);
                method.addInstruction(instruction);
            }
        }

        for (ExpandedComponentInstanceSymbol subComponent : componentSymbol.getSubComponents()) {
            String parameterString = "";
            for (ASTExpression var : subComponent.getArguments()) {
                System.out.println(var.toString());
                parameterString += getExpressionParameterConversion(var);
            }
            String result = "";
            result += GeneralHelperMethods.getTargetLanguageVariableInstanceName(subComponent.getName()) + ".init(" + parameterString + ");\n";

            TargetCodeInstruction instruction = new TargetCodeInstruction(result);
            method.addInstruction(instruction);
        }
    }

    public static void generateInitStaticVariablePart(Method method, Variable v, BluePrintCPP bluePrint) {
        //TODO add static variable filter function before generate init method
        //extract the static variable and their possible assignments
        //generate the static variable and their assignment if present
        //check if value is constant matrix/vector definition
        //fix this too
        VariableStatic variableStatic = (VariableStatic) v;

        if (!variableStatic.getAssignmentSymbol().isPresent()) {
            String instructionString = "";
            if (variableStatic.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getMatrixTypeName())) {
                instructionString = MathConverter.getMatrixInitLine(v, bluePrint);
            } else if (variableStatic.getVariableType().getTypeNameTargetLanguage().equals("double")) {
                instructionString = MathInformationRegister.getVariableInitName(v, bluePrint) + "=0;\n";
            } else if (variableStatic.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getRowVectorTypeName())) {
                instructionString = MathConverter.getRowVectorInitLine(v, bluePrint);
            } else if (variableStatic.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getColumnVectorTypeName())) {
                instructionString = MathConverter.getColumnVectorInitLine(v, bluePrint);
            }
            method.addInstruction(new TargetCodeInstruction(instructionString));
        } else {
            String instructionString = "";
            instructionString += variableStatic.getNameTargetLanguageFormat();
            instructionString += "=" + MathConverter.getConstantConversion(variableStatic.getAssignmentSymbol().get());
            instructionString += ";\n";
            method.addInstruction(new TargetCodeInstruction(instructionString));

        }
    }

    public static void generateInitNonStaticVariable(Method method, Variable v, BluePrintCPP bluePrint) {
        if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getMatrixTypeName())) {
            if (v.isParameterVariable()) {
                method.addInstruction(new TargetCodeInstruction("this->" + MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + MathInformationRegister.getVariableInitName(v, bluePrint) + ";\n"));
                method.addParameter(v);
            } else
                method.addInstruction(new TargetCodeInstruction(MathConverter.getMatrixInitLine(v, bluePrint)));
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getRowVectorTypeName())) {
            if (v.isParameterVariable()) {
                method.addInstruction(new TargetCodeInstruction("this->" + MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + MathInformationRegister.getVariableInitName(v, bluePrint) + ";\n"));
                method.addParameter(v);
            } else
                method.addInstruction(new TargetCodeInstruction(MathConverter.getRowVectorInitLine(v, bluePrint)));
        } else if (v.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getColumnVectorTypeName())) {
            if (v.isParameterVariable()) {
                method.addInstruction(new TargetCodeInstruction("this->" + MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + MathInformationRegister.getVariableInitName(v, bluePrint) + ";\n"));
                method.addParameter(v);
            } else
                method.addInstruction(new TargetCodeInstruction(MathConverter.getColumnVectorInitLine(v, bluePrint)));
        }
    }

    public static String getExpressionParameterConversion(ASTExpression var) {
        String parameterString = "";
        if (var.getSymbol().isPresent()) {
            boolean handled = false;
            MathExpressionSymbol symbol = (MathExpressionSymbol) var.getSymbol().get();
            if (symbol.isMatrixExpression()) {
                MathMatrixExpressionSymbol mathMatrixExpressionSymbol = (MathMatrixExpressionSymbol) symbol;
                if (mathMatrixExpressionSymbol.isValueExpression()) {
                    parameterString = MathConverter.getConstantConversion((MathMatrixArithmeticValueSymbol) mathMatrixExpressionSymbol);
                    handled = true;

                }
            }
            if (!handled)
                parameterString += symbol.getTextualRepresentation();
        } else {
            parameterString += var.toString();
        }
        return parameterString;
    }

    public static BluePrint convertComponentSymbolToBluePrint(ExpandedComponentInstanceSymbol componentSymbol, List<String> includeStrings, GeneratorCPP generatorCPP) {
        return convertComponentSymbolToBluePrint(componentSymbol, null, includeStrings, generatorCPP);
    }

    public static void fixMathFunctions(MathExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        MathFunctionFixer.fixMathFunctions(mathExpressionSymbol, bluePrintCPP);
    }


}
