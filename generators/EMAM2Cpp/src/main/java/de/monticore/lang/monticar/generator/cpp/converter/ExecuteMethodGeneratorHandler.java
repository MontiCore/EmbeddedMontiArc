/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math._symboltable.MathAssignmentOperator;
import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.VariableType;
import de.monticore.lang.monticar.generator.cpp.MathCommandRegisterCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.OctaveHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.MathChainedExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.MathStringExpression;
import de.monticore.lang.monticar.generator.cpp.viewmodel.Utils;
import de.monticore.lang.monticar.generator.optimization.MathInformationRegister;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
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
        } else if (mathValueExpressionSymbol.isBooleanExpression()) {
            return generateExecuteCode((MathBooleanExpressionSymbol) mathValueExpressionSymbol, includeStrings);
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
            if (mathValueSymbol.getValue() != null) {
                MathAssignmentExpressionSymbol assignment = new MathAssignmentExpressionSymbol();
                assignment.setNameOfMathValue(mathValueSymbol.getName());
                assignment.setExpressionSymbol(mathValueSymbol.getValue());
                assignment.setAssignmentOperator(new MathAssignmentOperator("="));
                result += type + " " + ExecuteMethodGenerator.generateExecuteCode(assignment, includeStrings);
            } else if (mathValueSymbol.getValue() == null) {
                result += type + " " + mathValueSymbol.getName();
                result += addInitializationString(mathValueSymbol, type, includeStrings);
                result += ";\n";
            }
        }
        ComponentConverter.currentBluePrint.getMathInformationRegister().addVariable(mathValueSymbol);
        //result += mathValueSymbol.getTextualRepresentation();
        return result;
    }

    public static String addInitializationString(MathValueSymbol mathValueSymbol, String typeString, List<String> includeStrings) {
        ASTElementType type = mathValueSymbol.getType().getType();

        String result = "";
        List<MathExpressionSymbol> dims = mathValueSymbol.getType().getDimensions();
        if (dims.size() == 1) {
            if (typeString.equals(TypeConverter.getColvecAccessString(type))) {
                result = "=" + TypeConverter.getDimensionString(TypeConverter.getColvecAccessString(type), dims, includeStrings);
            }
        } else if (dims.size() == 2) {
            if (typeIsCompatible(typeString, type, dims)) {
                result = "=" + TypeConverter.getDimensionString(TypeConverter.getMatAccessString(type), dims, includeStrings);
            }
        } else if (dims.size() == 3) {
            if (typeString.equals(TypeConverter.getCubeAccessString(type))) {
                result = "=" + TypeConverter.getDimensionString(TypeConverter.getCubeAccessString(type), dims, includeStrings);
            }
        }
        return result;
    }

    private static boolean typeIsCompatible(String typeString, ASTElementType type, List<MathExpressionSymbol> dims) {
        boolean result = typeString.equals(TypeConverter.getMatAccessString(type));
        if (!result && (dims.size() == 2) && (dims.get(1).getTextualRepresentation().contentEquals("1"))) {
            result = typeString.contentEquals("colvec") && TypeConverter.getMatAccessString(type).contentEquals("mat");
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
        } else if (mathValueType.getType().isWholeNumber()) {
            result = handleWholeNumberType(mathValueType);
        } else if (mathValueType.getType().isBoolean()) {
            result = handleBooleanType(mathValueType);
        } else {
            Log.info(mathValueType.getTextualRepresentation(), "Representation:");
            result = handleStructType(mathValueType);
        }
        return result;
    }

    private static String handleStructType(MathValueType mathValueType) {
        String res = "";
        if (mathValueType.getDimensions().size() == 0){
            if(mathValueType.getTypeRef().existsReferencedSymbol()){
                Symbol tmpSym = mathValueType.getTypeRef().getReferencedSymbol();
                if(tmpSym instanceof StructSymbol){
                    return tmpSym.getFullName().replace(".","_");
                }else{
                    Log.error("Referenced Symbol is not a Struct!");
                }
            }else{
                Log.error("Can not find Referenced type for " + mathValueType.getType().getName());
            }

        }else{
            Log.error("StructType: Case not handled!");
        }
        return res;
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

    public static String generateExecuteCodeFloatNumber(MathNumberExpressionSymbol mathNumberExpressionSymbol) {
        return Double.toString(mathNumberExpressionSymbol.getValue().getRealNumber().doubleValue());
    }

    public static String generateExecuteCode(MathBooleanExpressionSymbol mathBooleanExpressionSymbol, List<String> includeStrings) {
        return mathBooleanExpressionSymbol.getTextualRepresentation();
    }

    public static String generateExecuteCode(MathAssignmentExpressionSymbol mathAssignmentExpressionSymbol, List<String> includeStrings) {
        Log.info(mathAssignmentExpressionSymbol.getTextualRepresentation(), "mathAssignmentExpressionSymbol:");
        String result;
        if (mathAssignmentExpressionSymbol.getMathMatrixAccessOperatorSymbol() != null) {
            Log.info(mathAssignmentExpressionSymbol.getMathMatrixAccessOperatorSymbol().getTextualRepresentation(), "accessOperatorSymbol:");
            if (MathFunctionFixer.fixForLoopAccess(mathAssignmentExpressionSymbol.getNameOfMathValue(), ComponentConverter.currentBluePrint)) {

                result = mathAssignmentExpressionSymbol.getNameOfMathValue();
                result += ExecuteMethodGenerator.getCorrectAccessString(mathAssignmentExpressionSymbol.getNameOfMathValue(), mathAssignmentExpressionSymbol.getMathMatrixAccessOperatorSymbol(), includeStrings);
                result += mathAssignmentExpressionSymbol.getAssignmentOperator().getOperator() + " ";
                String input = ExecuteMethodGenerator.generateExecuteCode(mathAssignmentExpressionSymbol.getExpressionSymbol(), includeStrings) + ";\n";
                result += input;
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
        String assignment = mathAssignmentExpressionSymbol.getExpressionSymbol().getTextualRepresentation();
        Log.info(assignment, "assignment0:");
        if (assignmentSymbol instanceof MathMatrixNameExpressionSymbol) {
            MathMatrixNameExpressionSymbol matrixAssignmentSymbol = (MathMatrixNameExpressionSymbol) assignmentSymbol;
            if (useZeroBasedIndexing(matrixAssignmentSymbol)) {
                String matrixName = matrixAssignmentSymbol.getNameToAccess();
                String matrixAccess = ExecuteMethodGenerator.getCorrectAccessString(matrixAssignmentSymbol.getNameToAccess(), matrixAssignmentSymbol.getMathMatrixAccessOperatorSymbol(), includeStrings);
                assignment = String.format("%s%s", matrixName, matrixAccess);
                Log.info(assignment, "assignment1:");
            } else {
                assignment = ExecuteMethodGenerator.generateExecuteCode(assignmentSymbol, includeStrings);
                Log.info(assignment, "assignment2:");

            }
        } else {
            assignment = ExecuteMethodGenerator.generateExecuteCode(assignmentSymbol, includeStrings);
            Log.info(assignment, "assignment3:");
        }
        String result = String.format("%s %s %s;\n", name, op, assignment.trim());
        Log.info(name + " " + op + " " + assignment, "additionalInfo:");
        Log.info("result3: " + result, "MathAssignmentExpressionSymbol");
        return result;
    }

    private static boolean useZeroBasedIndexing(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol) {
        boolean isZeroBased = false;
        // test if array
        String name = mathMatrixNameExpressionSymbol.getNameToAccess();
        Variable variable = ComponentConverter.currentBluePrint.getVariable(name).orElse(null);
        if (!(variable != null && variable.isArray())) {
            if (MathConverter.curBackend.usesZeroBasedIndexing()) {
                if (!isFunctionCall(mathMatrixNameExpressionSymbol)) {
                    isZeroBased = true;
                }
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
        } else if (mathExpressionSymbol.getMathOperator().equals("/") && (mathExpressionSymbol.getLeftExpression() instanceof MathNumberExpressionSymbol) && (mathExpressionSymbol.getRightExpression() instanceof MathNumberExpressionSymbol)) {
            String left = ExecuteMethodGeneratorHandler.generateExecuteCodeFloatNumber((MathNumberExpressionSymbol) mathExpressionSymbol.getLeftExpression());
            String right = ExecuteMethodGeneratorHandler.generateExecuteCodeFloatNumber((MathNumberExpressionSymbol) mathExpressionSymbol.getRightExpression());
            result = String.format("%s%s%s", left, mathExpressionSymbol.getMathOperator(), right);
        } else {
            result += /*"("+*/  ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol.getLeftExpression(), includeStrings) + mathExpressionSymbol.getMathOperator();
            if (mathExpressionSymbol.getRightExpression() != null) {
                result += ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol.getRightExpression(), includeStrings);
            }
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
