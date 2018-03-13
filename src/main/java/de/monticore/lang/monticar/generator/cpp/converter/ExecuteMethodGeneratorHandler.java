package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.*;
import de.monticore.lang.monticar.generator.cpp.symbols.MathChainedExpression;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class ExecuteMethodGeneratorHandler {

    public static String generateExecuteCode(MathParenthesisExpressionSymbol mathExpressionSymbol, List<String> includeStrings) {
        String result = "";
        result += "(";
        result += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol.getMathExpressionSymbol(), includeStrings);
        result += ")";
        return result;
    }

    public static String generateExecuteCode(MathChainedExpression mathChainedExpression, List<String> includeStrings) {
        String result = "";
        result += ExecuteMethodGenerator.generateExecuteCode(mathChainedExpression.getFirstExpressionSymbol(), includeStrings);
        result += ExecuteMethodGenerator.generateExecuteCode(mathChainedExpression.getSecondExpressionSymbol(), includeStrings);
        return result;
    }

    public static String generateExecuteCode(MathStringExpression mathExpressionSymbol, List<String> includeStrings) {
        String result = "";
        result += mathExpressionSymbol.getTextualRepresentation();
        return result;
    }

    public static String generateExecuteCode(MathPreOperatorExpressionSymbol mathExpressionSymbol, List<String> includeStrings) {
        String result = "";
        result += mathExpressionSymbol.getOperator() + ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol.getMathExpressionSymbol(), includeStrings);
        return result;
    }

    public static String generateExecuteCode(MathCompareExpressionSymbol mathCompareExpressionSymbol, List<String> includeStrings) {
        String result = "";
        result += "(" + ExecuteMethodGenerator.generateExecuteCode(mathCompareExpressionSymbol.getLeftExpression(), includeStrings) + " " + mathCompareExpressionSymbol.getCompareOperator();
        result += " " + ExecuteMethodGenerator.generateExecuteCode(mathCompareExpressionSymbol.getRightExpression(), includeStrings) + ")";

        return result;
    }

    public static String generateExecuteCode(MathValueExpressionSymbol mathValueExpressionSymbol, List<String> includeStrings) {
        if (mathValueExpressionSymbol.isNameExpression()) {
            return generateExecuteCode((MathNameExpressionSymbol) mathValueExpressionSymbol, includeStrings);
        } else if (mathValueExpressionSymbol.isNumberExpression()) {
            return generateExecuteCode((MathNumberExpressionSymbol) mathValueExpressionSymbol, includeStrings);
        } else if (mathValueExpressionSymbol.isAssignmentDeclarationExpression()) {
            return generateExecuteCodeDeclaration((MathValueSymbol) mathValueExpressionSymbol, includeStrings);
        } else {
            Log.error("0xMAVAEXSY Case not handled!");
        }
        return null;
    }

    public static String generateExecuteCodeDeclaration(MathValueSymbol mathValueSymbol, List<String> includeStrings) {
        String result = "";
        List<String> properties = mathValueSymbol.getType().getProperties();
        if (properties.contains("static")) {
            Variable var = new Variable(mathValueSymbol.getName(), Variable.STATIC);
            var.setTypeNameTargetLanguage(TypeConverter.getVariableTypeNameForMathLanguageTypeName(mathValueSymbol.getType()));
            for (MathExpressionSymbol dimension : mathValueSymbol.getType().getDimensions())
                var.addDimensionalInformation(dimension.getTextualRepresentation());

            ComponentConverter.currentBluePrint.addVariable(var);
        } else {
            String type = generateExecuteCode(mathValueSymbol.getType(), includeStrings);
            result += type + " " + mathValueSymbol.getName();
            if (mathValueSymbol.getValue() != null) {
                result += " = " + ExecuteMethodGenerator.generateExecuteCode(mathValueSymbol.getValue(), includeStrings);
            } else if (mathValueSymbol.getValue() == null)
                result += addInitializationString(mathValueSymbol, type, includeStrings);
            result += ";\n";
        }
        ComponentConverter.currentBluePrint.getMathInformationRegister().addVariable(mathValueSymbol);
        //result += mathValueSymbol.getTextualRepresentation();
        return result;
    }

    public static String addInitializationString(MathValueSymbol mathValueSymbol, String typeString, List<String> includeStrings) {
        String result = "";
        List<MathExpressionSymbol> dims = mathValueSymbol.getType().getDimensions();
        if (dims.size() == 2) {
            MathExpressionSymbol rows = dims.get(0);
            MathExpressionSymbol cols = dims.get(1);

            if (typeString.equals(MathConverter.curBackend.getMatrixTypeName())) {
                result = "=" + MathConverter.curBackend.getMatrixTypeName() + "(" + ExecuteMethodGenerator.
                        generateExecuteCode(rows, includeStrings) + "," + ExecuteMethodGenerator.
                        generateExecuteCode(cols, includeStrings) + ")";
            }
        }
        return result;
    }

    public static String generateExecuteCode(MathValueSymbol mathValueSymbol, List<String> includeStrings) {
        String result = "";
        String type = generateExecuteCode(mathValueSymbol.getType(), includeStrings);
        result += type + " " + mathValueSymbol.getName();
        if (mathValueSymbol.getValue() != null) {
            result += " = " + ExecuteMethodGenerator.generateExecuteCode(mathValueSymbol.getValue(), includeStrings);
            result += ";\n";
        }
        //result += mathValueSymbol.getTextualRepresentation();
        return result;
    }

    public static String generateExecuteCode(MathValueType mathValueType, List<String> includeStrings) {
        String result = "";
        if (mathValueType.isRationalType()) {
            result = handleRationalType(mathValueType);
        } else if (mathValueType.getType().isIsWholeNumberNumber()) {
            result = handleWholeNumberType(mathValueType);
        } else if (mathValueType.getType().isIsBoolean()) {
            result = handleBooleanType(mathValueType);
        } else {
            Log.info(mathValueType.getTextualRepresentation(), "Representation:");
            Log.error("MathValueType: Case not handled!");
        }
        return result;
    }

    private static String handleRationalType(MathValueType mathValueType) {
        if (mathValueType.getDimensions().size() == 0)
            return "double";
        else if (mathValueType.getDimensions().size() == 2) {
            Log.info("Dim1:" + mathValueType.getDimensions().get(0).getTextualRepresentation() + "Dim2: " + mathValueType.getDimensions().get(1).getTextualRepresentation(), "DIMS:");
            if (mathValueType.getDimensions().get(0).getTextualRepresentation().equals("1")) {
                return MathConverter.curBackend.getRowVectorTypeName();
            } else if (mathValueType.getDimensions().get(1).getTextualRepresentation().equals("1")) {
                return MathConverter.curBackend.getColumnVectorTypeName();
            }
            return MathConverter.curBackend.getMatrixTypeName();//TODO improve in future
        } else {
            Log.error("0xGEEXCOMAVAT Type conversion Case not handled!");
        }
        return null;
    }

    private static String handleWholeNumberType(MathValueType mathValueType) {
        if (mathValueType.getDimensions().size() == 0)
            return "int";
        else if (mathValueType.getDimensions().size() == 2) {
            //TODO handle just like RationalMatrix right now
            Log.info("Dim1:" + mathValueType.getDimensions().get(0).getTextualRepresentation() + "Dim2: " + mathValueType.getDimensions().get(1).getTextualRepresentation(), "DIMS:");
            if (mathValueType.getDimensions().get(0).getTextualRepresentation().equals("1")) {
                return MathConverter.curBackend.getRowVectorTypeName();
            } else if (mathValueType.getDimensions().get(1).getTextualRepresentation().equals("1")) {
                return MathConverter.curBackend.getColumnVectorTypeName();
            }
            return MathConverter.curBackend.getMatrixTypeName();//TODO improve in future
        } else {
            Log.error("0xGEEXCOMAVAT Type conversion Case not handled!");
        }
        return null;
    }

    private static String handleBooleanType(MathValueType mathValueType) {
        if (mathValueType.getDimensions().size() == 0)
            return "bool";
        else {
            Log.error("0xGEEXCOMAVAT Type conversion Case not handled!");
        }
        return null;
    }

    public static String generateExecuteCode(MathNameExpressionSymbol mathNameExpressionSymbol, List<String> includeStrings) {
        Log.info(mathNameExpressionSymbol.getNameToResolveValue(), "NameToResolveValue:");
        return mathNameExpressionSymbol.getNameToResolveValue();
    }


    public static String generateExecuteCode(MathNumberExpressionSymbol mathNumberExpressionSymbol, List<String> includeStrings) {
        return mathNumberExpressionSymbol.getTextualRepresentation();
    }

    public static String generateExecuteCode(MathAssignmentExpressionSymbol mathAssignmentExpressionSymbol, List<String> includeStrings) {
        Log.info(mathAssignmentExpressionSymbol.getTextualRepresentation(), "mathAssignmentExpressionSymbol:");

        if (mathAssignmentExpressionSymbol.getMathMatrixAccessOperatorSymbol() != null) {
            if (MathFunctionFixer.fixForLoopAccess(mathAssignmentExpressionSymbol.getNameOfMathValue(), ComponentConverter.currentBluePrint)) {

                String result = mathAssignmentExpressionSymbol.getNameOfMathValue();
                result += ExecuteMethodGenerator.getCorrectAccessString(mathAssignmentExpressionSymbol.getNameOfMathValue(), mathAssignmentExpressionSymbol.getMathMatrixAccessOperatorSymbol(), includeStrings);
                result += mathAssignmentExpressionSymbol.getAssignmentOperator().getOperator() + " ";
                String input = ExecuteMethodGenerator.generateExecuteCode(mathAssignmentExpressionSymbol.getExpressionSymbol(), includeStrings) + ";\n";
                if (MathCommandRegisterCPP.containsCommandExpression(mathAssignmentExpressionSymbol.getExpressionSymbol(), input)) {
                    result += input;
                } else {
                    if(!StringValueListExtractorUtil.containsPortName(input))
                    result += StringIndexHelper.modifyContentBetweenBracketsByAdding(input, "-1");
                    else
                        result += input;
                }
                Log.info("result1: " + result, "MathAssignmentExpressionSymbol");
                return result;

            }
            /*if (mathAssignmentExpressionSymbol.getNameOfMathValue().equals("eigenVectors")) {
                for (Variable var : ComponentConverter.currentBluePrint.getMathInformationRegister().getVariables()) {
                    Log.info(var.getName(), "Var:");
                }
            }*/
            String result = mathAssignmentExpressionSymbol.getNameOfMathValue();
            result += ExecuteMethodGenerator.getCorrectAccessString(mathAssignmentExpressionSymbol.getNameOfMathValue(), mathAssignmentExpressionSymbol.getMathMatrixAccessOperatorSymbol(), includeStrings);
            result += mathAssignmentExpressionSymbol.getAssignmentOperator().getOperator() + " ";
            result += StringIndexHelper.modifyContentBetweenBracketsByAdding(ExecuteMethodGenerator.generateExecuteCode(mathAssignmentExpressionSymbol.getExpressionSymbol(), includeStrings) + ";\n", "-1");
            Log.info("result2: " + result, "MathAssignmentExpressionSymbol");

            return result;
        }
        String result = mathAssignmentExpressionSymbol.getNameOfMathValue() + " " + mathAssignmentExpressionSymbol.getAssignmentOperator().getOperator() + " " + ExecuteMethodGenerator.generateExecuteCode(mathAssignmentExpressionSymbol.getExpressionSymbol(), includeStrings) + ";\n";
        Log.info("result3: " + result, "MathAssignmentExpressionSymbol");
        return result;
    }


    public static String generateExecuteCode(MathForLoopExpressionSymbol mathForLoopExpressionSymbol, List<String> includeStrings) {
        String result = "";
        //For loop head
        result += generateExecuteCode(mathForLoopExpressionSymbol.getForLoopHead(), includeStrings);
        //For loop body
        result += "{\n";
        for (MathExpressionSymbol mathExpressionSymbol : mathForLoopExpressionSymbol.getForLoopBody())
            result += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, includeStrings);
        result += "}\n";

        return result;
    }

    public static String generateExecuteCode(MathForLoopHeadSymbol mathForLoopHeadSymbol, List<String> includeStrings) {
        String result = "";
        result += ForLoopHeadConverter.getForLoopHeadCode(mathForLoopHeadSymbol, includeStrings);
        return result;
    }

    public static String generateExecuteCode(MathArithmeticExpressionSymbol mathExpressionSymbol, List<String> includeStrings) {
        String result = "";
        if (mathExpressionSymbol.getMathOperator().equals("^")) {
            List<MathExpressionSymbol> list = new ArrayList<MathExpressionSymbol>();
            list.add(mathExpressionSymbol.getLeftExpression());
            list.add(mathExpressionSymbol.getRightExpression());
            String valueListString = "(" + OctaveHelper.getOctaveValueListString(list, ";") + ")";
            return MathConverter.curBackend.getPowerOfString(mathExpressionSymbol, valueListString, ";");
        } else {
            result += /*"("+*/  ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol.getLeftExpression(), includeStrings) + mathExpressionSymbol.getMathOperator();

            if (mathExpressionSymbol.getRightExpression() != null)
                result += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol.getRightExpression(), includeStrings);
        }
        return result;

    }


    public static String generateExecuteCode(MathConditionalExpressionsSymbol mathConditionalExpressionsSymbol, List<String> includeStrings) {
        String result = "";

        //if condition
        result += ExecuteMethodGeneratorHelper.generateIfConditionCode(mathConditionalExpressionsSymbol.getIfConditionalExpression(), includeStrings);
        //else if condition
        for (MathConditionalExpressionSymbol mathConditionalExpressionSymbol : mathConditionalExpressionsSymbol.getIfElseConditionalExpressions())
            result += "else " + ExecuteMethodGeneratorHelper.generateIfConditionCode(mathConditionalExpressionSymbol, includeStrings);
        //else block
        if (mathConditionalExpressionsSymbol.getElseConditionalExpression().isPresent()) {
            result += "else " + ExecuteMethodGeneratorHelper.generateIfConditionCode(mathConditionalExpressionsSymbol.getElseConditionalExpression().get(), includeStrings);
        }
        return result;
    }

}
