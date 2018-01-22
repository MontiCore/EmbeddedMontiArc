package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.*;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.OctaveHelper;
import de.monticore.lang.monticar.generator.cpp.symbols.MathChainedExpression;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class ExecuteMethodGenerator {
    public static String generateExecuteCode(MathExpressionSymbol mathExpressionSymbol, List<String> includeStrings) {
        String result = null;
        //Not needed for execute method
        /*if (mathExpressionSymbol.isAssignmentDeclarationExpression()) {
            return generateExecuteCode((MathValueSymbol) mathExpressionSymbol, includeStrings);
        } */
        if (mathExpressionSymbol == null) {

        } else if (mathExpressionSymbol.isAssignmentExpression()) {
            result = ExecuteMethodGeneratorHandler.generateExecuteCode((MathAssignmentExpressionSymbol) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.isCompareExpression()) {
            result = ExecuteMethodGeneratorHandler.generateExecuteCode((MathCompareExpressionSymbol) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.isForLoopExpression()) {
            result = ExecuteMethodGeneratorHandler.generateExecuteCode((MathForLoopExpressionSymbol) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.isMatrixExpression()) {
            result = ExecuteMethodGeneratorMatrixExpressionHandler.generateExecuteCode((MathMatrixExpressionSymbol) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.isConditionalsExpression()) {
            //This is the real conditional expressions, the if/elseif/else statement is
            // called conditional expression in the math language
            //name in math langugae could be changed to avoid confusion
            result = ExecuteMethodGeneratorHandler.generateExecuteCode((MathConditionalExpressionsSymbol) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.isConditionalExpression()) {
            Log.error("ConditionalExpression should not be handled this way!");
            //return ExecuteMethodGeneratorHandler.generateExecuteCode((MathConditionalExpressionSymbol) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.isArithmeticExpression()) {
            result = ExecuteMethodGeneratorHandler.generateExecuteCode((MathArithmeticExpressionSymbol) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.isValueExpression()) {
            result = ExecuteMethodGeneratorHandler.generateExecuteCode((MathValueExpressionSymbol) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.isMathValueTypeExpression()) {
            result = ExecuteMethodGeneratorHandler.generateExecuteCode((MathValueType) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.isPreOperatorExpression()) {
            result = ExecuteMethodGeneratorHandler.generateExecuteCode((MathPreOperatorExpressionSymbol) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.getExpressionID() == MathStringExpression.ID) {
            result = ExecuteMethodGeneratorHandler.generateExecuteCode((MathStringExpression) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.getExpressionID() == MathChainedExpression.ID) {
            result = ExecuteMethodGeneratorHandler.generateExecuteCode((MathChainedExpression) mathExpressionSymbol, includeStrings);
        } else if (mathExpressionSymbol.isParenthesisExpression()) {
            result = ExecuteMethodGeneratorHandler.generateExecuteCode((MathParenthesisExpressionSymbol) mathExpressionSymbol, includeStrings);
        } else {
            Log.info(mathExpressionSymbol.getTextualRepresentation(), "Symbol:");
            Log.error("Case not handled!");
        }
        return result;
    }

    /**
     * Returns correct string for generation of matrix/array access
     *
     * @return
     */
    public static String getCorrectAccessString(String nameOfMathValue, MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol, List<String> includeStrings) {
        String result = "";
        if (ComponentConverterMethodGeneration.currentComponentSymbol.getPort(nameOfMathValue + "[1]").isPresent()) {
            result += ExecuteMethodGeneratorMatrixExpressionHandler.generateExecuteCode(mathMatrixAccessOperatorSymbol, includeStrings, true).replaceAll("\\(", "\\[").replaceAll("\\)", "\\]") + " ";
        } else {
            result += ExecuteMethodGeneratorMatrixExpressionHandler.generateExecuteCode(mathMatrixAccessOperatorSymbol, includeStrings, true) + " ";
        }
        return result;
    }

/*
    public static String generateExecuteCodeFixForLoopAccess(MathMatrixAccessSymbol mathMatrixAccessSymbol, List<String> includeStrings) {
        String result = "";

        if (mathMatrixAccessSymbol.isDoubleDot())
            result += ":";
        else {
            MathFunctionFixer.fixMathFunctions(mathMatrixAccessSymbol.getMathExpressionSymbol().get(), currentBluePrint);
            result += generateExecuteCode(mathMatrixAccessSymbol.getMathExpressionSymbol().get(), includeStrings);
            //result += "-1";
        }
        return result;
    }*/

}
