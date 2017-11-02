package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math.math._symboltable.expression.IArithmeticExpression;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.*;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.OctaveHelper;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class ExecuteMethodGeneratorMatrixExpressionHandler {


    public static String generateExecuteCode(MathMatrixArithmeticExpressionSymbol mathMatrixArithmeticExpressionSymbol, List<String> includeStrings) {
        String result = "";
        if (mathMatrixArithmeticExpressionSymbol.getMathOperator().equals("^")) {
            result = generateExecuteCodeMatrixPowerOfOperator(mathMatrixArithmeticExpressionSymbol, includeStrings);
        } else if (mathMatrixArithmeticExpressionSymbol.getMathOperator().equals(".^")) {
            result = generateExecuteCodeMatrixEEPowerOf(mathMatrixArithmeticExpressionSymbol, includeStrings);
        } else if (mathMatrixArithmeticExpressionSymbol.getMathOperator().equals("./")) {
            result = generateExecuteCodeMatrixEEDivide(mathMatrixArithmeticExpressionSymbol, includeStrings);
        /*} else if (mathArithmeticExpressionSymbol.getMathOperator().equals("./")) {
            Log.error("reace");
            result += "\"ldivide\"";
            includeStrings.add("Helper");
        */
        } else {

            result += /*"(" +*/ ExecuteMethodGenerator.generateExecuteCode(mathMatrixArithmeticExpressionSymbol.getLeftExpression(), includeStrings) + " " + mathMatrixArithmeticExpressionSymbol.getMathOperator();

            if (mathMatrixArithmeticExpressionSymbol.getRightExpression() != null)
                result += " " + ExecuteMethodGenerator.generateExecuteCode(mathMatrixArithmeticExpressionSymbol.getRightExpression(), includeStrings);
        }
        /*result += ")"*/

        return result;
    }

    public static String calculateValueListString(IArithmeticExpression mathExpressionSymbol) {
        List<MathExpressionSymbol> list = new ArrayList<MathExpressionSymbol>();
        list.add(mathExpressionSymbol.getLeftExpression());
        list.add(mathExpressionSymbol.getRightExpression());
        return "(" + OctaveHelper.getOctaveValueListString(list) + ")";
    }

    public static String generateExecuteCodeMatrixEEDivide(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, List<String> includeStrings) {
        String valueListString = calculateValueListString(mathExpressionSymbol);
        return OctaveHelper.getCallOctaveFunctionFirstResult(mathExpressionSymbol.getLeftExpression(), "ldivide", valueListString, false);
    }

    public static String generateExecuteCodeMatrixEEPowerOf(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, List<String> includeStrings) {
        String valueListString = calculateValueListString(mathExpressionSymbol);
        return OctaveHelper.getCallOctaveFunctionFirstResult(mathExpressionSymbol.getLeftExpression(), "power", valueListString, false);
    }

    public static String generateExecuteCodeMatrixPowerOfOperator(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, List<String> includeStrings) {
        String valueListString = calculateValueListString(mathExpressionSymbol);
        return OctaveHelper.getCallBuiltInFunctionFirstResult(mathExpressionSymbol.getLeftExpression(), "Fmpower", valueListString, false, 1);
    }

    public static String generateExecuteCode(MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol, List<String> includeStrings, boolean setMinusOne) {
        String result = "";
        int counter = 0;
        String matrixExtractionPart = calculateMatrixExtractionPart(mathMatrixAccessOperatorSymbol);
        result += matrixExtractionPart;
        result += mathMatrixAccessOperatorSymbol.getAccessStartSymbol();
        result += updateMatrixAccessString(mathMatrixAccessOperatorSymbol, counter, matrixExtractionPart, setMinusOne, includeStrings);

        result += mathMatrixAccessOperatorSymbol.getAccessEndSymbol();
        return result;
    }

    public static String calculateMatrixExtractionPart(MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol) {
        String result = "";
        if (mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().size() == 2) {
            if (mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().get(0).isDoubleDot()) {
                result += ".column";
            } else if (mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().get(1).isDoubleDot()) {
                result += ".row";
            }
        }
        return result;
    }

    public static boolean isColumnString(String matrixExtractionPart) {
        return matrixExtractionPart.equals(".column");
    }

    public static boolean isRowString(String matrixExtractionPart) {
        return matrixExtractionPart.equals(".row");
    }

    public static int getIgnoreCounterAt(String matrixExtractionPart) {
        int ignoreCounterAt = -1;
        if (isColumnString(matrixExtractionPart)) {
            ignoreCounterAt = 0;
        } else if (isRowString(matrixExtractionPart)) {
            ignoreCounterAt = 1;
        }
        return ignoreCounterAt;
    }

    public static int getCounterSetMinusOne(String matrixExtractionPart) {
        int counterSetMinusOne = -1;
        if (isColumnString(matrixExtractionPart)) {
            counterSetMinusOne = 1;
        } else if (isRowString(matrixExtractionPart)) {
            counterSetMinusOne = 0;
        }
        return counterSetMinusOne;
    }

    public static String updateMatrixAccessString(MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol, int counter, String matrixExtractionPart, boolean setMinusOne, List<String> includeStrings) {
        int ignoreCounterAt = getIgnoreCounterAt(matrixExtractionPart);
        int counterSetMinusOne = getCounterSetMinusOne(matrixExtractionPart);
        String result = "";
        for (MathMatrixAccessSymbol mathMatrixAccessSymbol : mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols()) {
            if (counter == ignoreCounterAt) {
                ++counter;
            } else {
                if (counter == counterSetMinusOne) {
                    result += generateExecuteCode(mathMatrixAccessSymbol, includeStrings, true);
                } else
                    result += generateExecuteCode(mathMatrixAccessSymbol, includeStrings, setMinusOne);
                ++counter;
                if (counter < mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().size() && counter != ignoreCounterAt) {
                    result += ", ";
                }
            }
        }
        return result;
    }

    public static String generateExecuteCode(MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol, List<String> includeStrings) {
        String result = "";
        int counter = 0;

        String matrixExtractionPart = calculateMatrixExtractionPart(mathMatrixAccessOperatorSymbol);
        result += matrixExtractionPart;
        result += mathMatrixAccessOperatorSymbol.getAccessStartSymbol();

        if (MathFunctionFixer.fixForLoopAccess(mathMatrixAccessOperatorSymbol.getMathMatrixNameExpressionSymbol(), ComponentConverter.currentBluePrint)) {
            result += updateMatrixAccessStringFixForLoop(mathMatrixAccessOperatorSymbol, counter, matrixExtractionPart, includeStrings);
        } else {
            result += updateMatrixAccessString(mathMatrixAccessOperatorSymbol, counter, matrixExtractionPart, false, includeStrings);
        }
        result += mathMatrixAccessOperatorSymbol.getAccessEndSymbol();
        return result;
    }

    public static String updateMatrixAccessStringFixForLoop(MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol, int counter, String matrixExtractionPart, List<String> includeStrings) {
        int ignoreCounterAt = getIgnoreCounterAt(matrixExtractionPart);
        String result = "";
        for (MathMatrixAccessSymbol mathMatrixAccessSymbol : mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols()) {
            if (counter == ignoreCounterAt) {
                ++counter;
            } else {
                result += generateExecuteCode(mathMatrixAccessSymbol, includeStrings, true);
                ++counter;
                if (counter < mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().size() && counter != ignoreCounterAt) {
                    result += ", ";
                }
            }
        }
        return result;
    }

    public static String generateExecuteCode(MathMatrixAccessSymbol mathMatrixAccessSymbol, List<String> includeStrings, boolean setMinusOne) {
        String result = "";

        if (mathMatrixAccessSymbol.isDoubleDot())
            result += ":";
        else
            result += ExecuteMethodGenerator.generateExecuteCode(mathMatrixAccessSymbol.getMathExpressionSymbol().get(), includeStrings);
        if (setMinusOne)
            result += "-1";
        return result;
    }

    public static String generateExecuteCode(MathMatrixAccessSymbol mathMatrixAccessSymbol, List<String> includeStrings) {
        return generateExecuteCode(mathMatrixAccessSymbol, includeStrings, false);
    }


    public static String generateExecuteCode(MathMatrixExpressionSymbol mathMatrixExpressionSymbol, List<String> includeStrings) {
        String result = "";


        if (mathMatrixExpressionSymbol.isMatrixArithmeticExpression()) {
            result = generateExecuteCode((MathMatrixArithmeticExpressionSymbol) mathMatrixExpressionSymbol, includeStrings);
        } else if (mathMatrixExpressionSymbol.isMatrixAccessExpression()) {
            result = generateExecuteCode((MathMatrixAccessSymbol) mathMatrixExpressionSymbol, includeStrings);
        } else if (mathMatrixExpressionSymbol.isMatrixNameExpression()) {
            result = generateExecuteCode((MathMatrixNameExpressionSymbol) mathMatrixExpressionSymbol, includeStrings);
        } else if (mathMatrixExpressionSymbol.isValueExpression()) {
            result = generateExecuteCode((MathMatrixArithmeticValueSymbol) mathMatrixExpressionSymbol, includeStrings);
        } else {
            Log.info(mathMatrixExpressionSymbol.getTextualRepresentation(), "Symbol:");
            Log.info(mathMatrixExpressionSymbol.getClass().getName(), "Symbol Name:");
            Log.error("0xMAMAEXSY Case not handled!");
        }
        return result;
    }

    public static String generateExecuteCode(MathMatrixArithmeticValueSymbol mathMatrixArithmeticValueSymbol, List<String> includeStrings) {
        return MathConverter.getConstantConversion(mathMatrixArithmeticValueSymbol);
    }

    public static String generateExecuteCode(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol, List<String> includeStrings) {
        String result = "";
        //TODO fix matrix access parameter stuff
        result += mathMatrixNameExpressionSymbol.getNameToAccess();
        if (mathMatrixNameExpressionSymbol.hasMatrixAccessExpression()) {
            mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().setMathMatrixNameExpressionSymbol(mathMatrixNameExpressionSymbol);
            result += generateExecuteCode(mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol(), includeStrings);
        }
        return result;
    }
}
