package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.MathCommandRegisterCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.OctaveHelper;
import de.monticore.lang.monticar.generator.cpp.StringValueListExtractorUtil;
import de.monticore.lang.monticar.generator.cpp.symbols.MathChainedExpression;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
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
        ASTElementType type = mathValueSymbol.getType().getType();
        String matString;
        String colvecString;
        String cubeString;

        if(type.isIsRational()){
            matString = MathConverter.curBackend.getMatrixTypeName();
            colvecString = MathConverter.curBackend.getColumnVectorTypeName();
            cubeString = MathConverter.curBackend.getCubeTypeName();
        }else if(type.isIsWholeNumberNumber()){
            matString = MathConverter.curBackend.getWholeNumberMatrixTypeName();
            colvecString = MathConverter.curBackend.getWholeNumberColumnVectorTypeName();
            cubeString = MathConverter.curBackend.getWholeNumberCubeTypeName();
        }else{
            Log.error("Initialization not handled!");
            return "";
        }

        String result = "";
        List<MathExpressionSymbol> dims = mathValueSymbol.getType().getDimensions();
        if (dims.size() == 1) {
            MathExpressionSymbol rows = dims.get(0);
            if (typeString.equals(colvecString)) {
                result = "=" + colvecString + "(" + ExecuteMethodGenerator.
                        generateExecuteCode(rows, includeStrings) + ")";
            }
        } else if (dims.size() == 2) {
            MathExpressionSymbol rows = dims.get(0);
            MathExpressionSymbol cols = dims.get(1);

            if (typeString.equals(matString)) {
                result = "=" + matString + "(" + ExecuteMethodGenerator.
                        generateExecuteCode(rows, includeStrings) + "," + ExecuteMethodGenerator.
                        generateExecuteCode(cols, includeStrings) + ")";
            }
        } else if (dims.size() == 3) {
            MathExpressionSymbol rows = dims.get(0);
            MathExpressionSymbol cols = dims.get(1);
            MathExpressionSymbol slices = dims.get(2);

            if (typeString.equals(cubeString)) {
                result = "=" + cubeString + "(" + ExecuteMethodGenerator.
                        generateExecuteCode(rows, includeStrings) + "," + ExecuteMethodGenerator.
                        generateExecuteCode(cols, includeStrings) + "," + ExecuteMethodGenerator.
                        generateExecuteCode(slices, includeStrings) + ")";
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
        if (mathValueType.getDimensions().size() == 0) {
            return "double";
        } else if (mathValueType.getDimensions().size() == 1) {
            Log.info("Dim1:" + mathValueType.getDimensions().get(0).getTextualRepresentation(), "DIMS:");
            return MathConverter.curBackend.getColumnVectorTypeName();
        } else if (mathValueType.getDimensions().size() == 2) {
            Log.info("Dim1:" + mathValueType.getDimensions().get(0).getTextualRepresentation() + "Dim2: " + mathValueType.getDimensions().get(1).getTextualRepresentation(), "DIMS:");
            if (mathValueType.getDimensions().get(0).getTextualRepresentation().equals("1")) {
                return MathConverter.curBackend.getRowVectorTypeName();
            } else if (mathValueType.getDimensions().get(1).getTextualRepresentation().equals("1")) {
                return MathConverter.curBackend.getColumnVectorTypeName();
            }
            return MathConverter.curBackend.getMatrixTypeName();//TODO improve in future
        } else if (mathValueType.getDimensions().size() == 3) {
            Log.info("Dim1:" + mathValueType.getDimensions().get(0).getTextualRepresentation() + "Dim2: " + mathValueType.getDimensions().get(1).getTextualRepresentation() + "Dim3: " + mathValueType.getDimensions().get(2).getTextualRepresentation(), "DIMS:");
            return MathConverter.curBackend.getCubeTypeName();
        } else {
            Log.error("0xGEEXCOMAVAT Type conversion Case not handled!");
        }
        return null;
    }

    private static String handleWholeNumberType(MathValueType mathValueType) {
        if (mathValueType.getDimensions().size() == 0) {
            return "int";
        } else if (mathValueType.getDimensions().size() == 1) {
            return MathConverter.curBackend.getWholeNumberColumnVectorTypeName();
        } else if (mathValueType.getDimensions().size() == 2) {
            Log.info("Dim1:" + mathValueType.getDimensions().get(0).getTextualRepresentation() + "Dim2: " + mathValueType.getDimensions().get(1).getTextualRepresentation(), "DIMS:");
            if (mathValueType.getDimensions().get(0).getTextualRepresentation().equals("1")) {
                return MathConverter.curBackend.getWholeNumberRowVectorTypeName();
            } else if (mathValueType.getDimensions().get(1).getTextualRepresentation().equals("1")) {
                return MathConverter.curBackend.getWholeNumberColumnVectorTypeName();
            }
            return MathConverter.curBackend.getWholeNumberMatrixTypeName();
        } else if (mathValueType.getDimensions().size() == 3) {
            return MathConverter.curBackend.getWholeNumberCubeTypeName();
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
        String result;
        if (mathAssignmentExpressionSymbol.getMathMatrixAccessOperatorSymbol() != null) {
            Log.info(mathAssignmentExpressionSymbol.getMathMatrixAccessOperatorSymbol().getTextualRepresentation(),"accessOperatorSymbol:");
            if (MathFunctionFixer.fixForLoopAccess(mathAssignmentExpressionSymbol.getNameOfMathValue(), ComponentConverter.currentBluePrint)) {

                result = mathAssignmentExpressionSymbol.getNameOfMathValue();
                result += ExecuteMethodGenerator.getCorrectAccessString(mathAssignmentExpressionSymbol.getNameOfMathValue(), mathAssignmentExpressionSymbol.getMathMatrixAccessOperatorSymbol(), includeStrings);
                result += mathAssignmentExpressionSymbol.getAssignmentOperator().getOperator() + " ";
                String input = ExecuteMethodGenerator.generateExecuteCode(mathAssignmentExpressionSymbol.getExpressionSymbol(), includeStrings) + ";\n";
                if (MathCommandRegisterCPP.containsCommandExpression(mathAssignmentExpressionSymbol.getExpressionSymbol(), input)) {
                    result += input;
                } else {
                    if (!StringValueListExtractorUtil.containsPortName(input))
                        result += StringIndexHelper.modifyContentBetweenBracketsByAdding(input, "-1");
                    else
                        result += input;
                }
                Log.info("result1: " + result, "MathAssignmentExpressionSymbol");
            } else {
                /*if (mathAssignmentExpressionSymbol.getNameOfMathValue().equals("eigenVectors")) {
                for (Variable var : ComponentConverter.currentBluePrint.getMathInformationRegister().getVariables()) {
                    Log.info(var.getName(), "Var:");
                }
            }*/

                result = mathAssignmentExpressionSymbol.getNameOfMathValue();
                result += ExecuteMethodGenerator.getCorrectAccessString(mathAssignmentExpressionSymbol.getNameOfMathValue(), mathAssignmentExpressionSymbol.getMathMatrixAccessOperatorSymbol(), includeStrings);
                result += mathAssignmentExpressionSymbol.getAssignmentOperator().getOperator() + " ";
                result += StringIndexHelper.modifyContentBetweenBracketsByAdding(ExecuteMethodGenerator.generateExecuteCode(mathAssignmentExpressionSymbol.getExpressionSymbol(), includeStrings) + ";\n", "-1");
                Log.info("result2: " + result, "MathAssignmentExpressionSymbol");
            }
        } else {
            result = generateExecuteCodeForNonMatrixElementAssignments(mathAssignmentExpressionSymbol, includeStrings);
        }
        return result;
    }

    private static String generateExecuteCodeForNonMatrixElementAssignments(MathAssignmentExpressionSymbol mathAssignmentExpressionSymbol, List<String> includeStrings) {
        String name = mathAssignmentExpressionSymbol.getNameOfMathValue();
        String op = mathAssignmentExpressionSymbol.getAssignmentOperator().getOperator();
        MathExpressionSymbol assignmentSymbol = mathAssignmentExpressionSymbol.getExpressionSymbol().getRealMathExpressionSymbol();
        String assignment=mathAssignmentExpressionSymbol.getExpressionSymbol().getTextualRepresentation();
        Log.info(assignment,"assignment0:");
        if (assignmentSymbol instanceof MathMatrixNameExpressionSymbol) {
            MathMatrixNameExpressionSymbol matrixAssignmentSymbol = (MathMatrixNameExpressionSymbol) assignmentSymbol;
            if (useZeroBasedIndexing(matrixAssignmentSymbol)) {
                String matrixName = matrixAssignmentSymbol.getNameToAccess();
                String matrixAccess = ExecuteMethodGenerator.getCorrectAccessString(matrixAssignmentSymbol.getNameToAccess(), matrixAssignmentSymbol.getMathMatrixAccessOperatorSymbol(), includeStrings);
                assignment = String.format("%s%s", matrixName, matrixAccess);
                Log.info(assignment,"assignment1:");
            } else {
                assignment = ExecuteMethodGenerator.generateExecuteCode(assignmentSymbol, includeStrings);
                Log.info(assignment,"assignment2:");

            }
        } else {
            assignment = ExecuteMethodGenerator.generateExecuteCode(assignmentSymbol, includeStrings);
            Log.info(assignment,"assignment3:");

        }
        String result = String.format("%s %s %s;\n", name, op, assignment);
        Log.info(name + " " + op + " " + assignment, "additionalInfo:");
        Log.info("result3: " + result, "MathAssignmentExpressionSymbol");
        return result;
    }

    private static boolean useZeroBasedIndexing(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol) {
        boolean isZeroBased = false;
        if (MathConverter.curBackend.usesZeroBasedIndexing()) {
            if (!isFunctionCall(mathMatrixNameExpressionSymbol)) {
                isZeroBased = true;
            }
        }
        return isZeroBased;
    }

    private static boolean isFunctionCall(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol) {
        boolean isFunctionCall = false;
        if (MathCommandRegisterCPP.containsCommandExpression(mathMatrixNameExpressionSymbol, mathMatrixNameExpressionSymbol.getTextualRepresentation())) {
            isFunctionCall = true;
        }
        return isFunctionCall;
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
