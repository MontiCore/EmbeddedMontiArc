/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathParenthesisExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class OctaveHelper {
    public static String fileName = "Helper";

    public static FileContent getOctaveHelperFileContent() {
        FileContent fileContent = new FileContent();
        fileContent.setFileName(fileName + ".h");
        String fileContentString = OctaveHelperSource.octaveHelperSourceCode;

        fileContent.setFileContent(fileContentString);
        return fileContent;
    }

    public static String getOctaveValueListString(List<MathExpressionSymbol> expressionSymbolList, String separator) {
        String valueListString = "";
        int counter = 0;
        for (MathExpressionSymbol symbol : expressionSymbolList) {
//            MathFunctionFixer.fixMathFunctions(symbol, ComponentConverter.currentBluePrint);
            valueListString += ExecuteMethodGenerator.generateExecuteCode(symbol, new ArrayList<String>());
            ++counter;
            if (counter < expressionSymbolList.size())
                valueListString += separator;
        }
        return valueListString;
    }

    public static String getOctaveValueListString(List<MathExpressionSymbol> expressionSymbolList) {
        return getOctaveValueListString(expressionSymbolList, ",");

    }

    public static String getCallBuiltInFunctionFirstResult(MathExpressionSymbol mathExpressionSymbol, String functionName, String valueListString, boolean useBraces, int argsOut) {
        return getCallBuiltInFunction(mathExpressionSymbol, functionName, valueListString, "FirstResult", useBraces, argsOut);
    }

    public static String getCallOctaveFunctionFirstResult(MathExpressionSymbol mathExpressionSymbol, String functionName, String valueListString, boolean useBraces) {
        return getCallOctaveFunction(mathExpressionSymbol, functionName, valueListString, "FirstResult", useBraces);
    }

    public static String getCallOctaveFunctionSecondResult(MathExpressionSymbol mathExpressionSymbol, String functionName, String valueListString, boolean useBraces) {
        return getCallOctaveFunction(mathExpressionSymbol, functionName, valueListString, "SecondResult", useBraces);
    }


    /*public static String getCallOctaveFunctionThirdResult(MathExpressionSymbol mathExpressionSymbol, String functionName, String valueListString, boolean useBraces) {
        return getCallOctaveFunction(mathExpressionSymbol, functionName, valueListString, "ThirdResult", useBraces);
    }

    public static String getCallOctaveFunctionFourthResult(MathExpressionSymbol mathExpressionSymbol, String functionName, String valueListString, boolean useBraces) {
        return getCallOctaveFunction(mathExpressionSymbol, functionName, valueListString, "FourthResult", useBraces);
    }*/

    /*public static String getCallOctaveFunction(MathExpressionSymbol mathExpressionSymbol, String functionName, String valueListString, String numberResultString) {

        String functionTypeName = OctaveHelper.getOctaveFunctionTypeName(mathExpressionSymbol);
        return getCallOctaveFunction(mathExpressionSymbol, functionName, functionTypeName, valueListString, numberResultString);
    }*/

    public static String getCallOctaveFunction(MathExpressionSymbol mathExpressionSymbol, String functionName, String valueListString, String numberResultString, boolean useBraces) {

        String functionTypeName = OctaveHelper.getOctaveFunctionTypeName(mathExpressionSymbol);
        return getCallOctaveFunction(mathExpressionSymbol, functionName, functionTypeName, valueListString, numberResultString, useBraces);
    }

    public static String getCallBuiltInFunction(MathExpressionSymbol mathExpressionSymbol, String functionName, String valueListString, String numberResultString, boolean useBraces, int argsOut) {

        String functionTypeName = OctaveHelper.getOctaveFunctionTypeName(mathExpressionSymbol);
        return getCallBuiltInFunction(functionName, functionTypeName, valueListString, numberResultString, argsOut);
    }


    public static String getCallBuiltInFunction(String functionName, String functionType, String valueListString, String numberResultString, int argsOut) {
        String result = "";
        result += "Helper::get";
        //TODO calc type of function to call
        result += functionType;
        result += "FromOctaveList";
        result += numberResultString;
        result += "(";
        result += functionName;
        result += "(Helper::convertToOctaveValueList";
        result += valueListString;
        result += "," + argsOut;
        result += "))";
        // result += ",\"" + functionName + "\")";
        return result;
    }

    public static String getCallOctaveFunction(MathExpressionSymbol mathExpressionSymbol, String functionName, String functionType, String valueListString, String numberResultString, boolean useBraces) {
        return getCallOctaveFunction(mathExpressionSymbol, functionName, functionType, valueListString, numberResultString, useBraces, 1);
    }

    public static String getCallOctaveFunction(MathExpressionSymbol mathExpressionSymbol, String functionName, String functionType, String valueListString, String numberResultString, boolean useBraces, int argsOut) {
        String result = "";
        result += "Helper::callOctaveFunction";
        //TODO calc type of function to call
        result += functionType;
        result += numberResultString;
        result += "(Helper::convertToOctaveValueList";
        if (useBraces)
            result += "({";
        result += valueListString;
        if (useBraces)
            result += "})";
        result += "";
        result += ",\"" + functionName + "\"";
        result += "," + argsOut;
        result += ")";
        return result;
    }

    /*public static String getCallOctaveFunction(MathExpressionSymbol mathExpressionSymbol, String functionName, String functionType, String valueListString, String numberResultString) {
        return getCallOctaveFunction(mathExpressionSymbol, functionName, functionType, valueListString, numberResultString, true);
    }*/

    public static String getOctaveFunctionTypeName(MathExpressionSymbol mathExpressionSymbol) {
        if (mathExpressionSymbol.isMatrixExpression()) {
            return getOctaveFunctionTypeName((MathMatrixExpressionSymbol) mathExpressionSymbol);
        } else if (mathExpressionSymbol.isArithmeticExpression()) {
            return getOctaveFunctionTypeName((MathArithmeticExpressionSymbol) mathExpressionSymbol);
        } else if (mathExpressionSymbol.isValueExpression()) {
            return getOctaveFunctionTypeName((MathValueExpressionSymbol) mathExpressionSymbol);
        } else if (mathExpressionSymbol.isParenthesisExpression()) {
            return getOctaveFunctionTypeName(((MathParenthesisExpressionSymbol) mathExpressionSymbol).getMathExpressionSymbol());
        } else {
            Log.info(mathExpressionSymbol.getClass().getName(), "Symbol Name:");
            Log.info(mathExpressionSymbol.getTextualRepresentation(), "Symbol:");
            Log.error("Octave function type name not handled");
            return null;
        }
    }

    public static String getOctaveFunctionTypeName(MathValueExpressionSymbol mathExpressionSymbol) {
        if (mathExpressionSymbol.isNumberExpression())
            return "Double";
        else if (mathExpressionSymbol.isNameExpression()) {
            Variable variable = ComponentConverter.currentBluePrint.getMathInformationRegister().getVariable(((MathNameExpressionSymbol) mathExpressionSymbol).getNameToResolveValue());
            String typeName = variable.getVariableType().getTypeNameTargetLanguage();
            return typeName.substring(0, 1).toUpperCase() + typeName.substring(1);
        } else {

            Log.debug("Not supported yet","Case not handled OctaveHelper");
        }
        return null;
    }

    public static String getOctaveFunctionTypeName(MathArithmeticExpressionSymbol mathExpressionSymbol) {
        return getOctaveFunctionTypeName(mathExpressionSymbol.getLeftExpression());
    }

    public static String getOctaveFunctionTypeName(MathMatrixExpressionSymbol mathExpressionSymbol) {
        if (mathExpressionSymbol.isMatrixNameExpression()) {
            return getOctaveFunctionTypeName((MathMatrixNameExpressionSymbol) mathExpressionSymbol);
        } else {
            Log.debug("Not supported yet","Case not handled OctaveHelper");
            return null;
        }
    }


    public static String getOctaveFunctionTypeName(MathMatrixNameExpressionSymbol mathExpressionSymbol) {
        String result = "";

        if (mathExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size() == 2) {
            MathMatrixAccessSymbol firstAccessSymbol = mathExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(0);
            MathMatrixAccessSymbol secondAccessSymbol = mathExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(1);


            if (firstAccessSymbol.isDoubleDot()) {
                result += "ColumnVector";
            } else if (secondAccessSymbol.isDoubleDot()) {
                result += "RowVector";
            } else {
                result += "Double";
            }


        } else if (mathExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size() == 1) {

        } else {
            Log.info(mathExpressionSymbol.getTextualRepresentation(), "Symbol:");
            Log.debug("FunctionTypeName not handled!", "Case not handled");
        }

        return result;
    }

    public static String getCallBuiltInFunction(MathExpressionSymbol mathExpressionSymbol, String functionName, String functionType, String valueListString, String numberResult, boolean b, int argsOut) {
        return getCallBuiltInFunction(functionName, functionType, valueListString, numberResult, argsOut);
    }
}
