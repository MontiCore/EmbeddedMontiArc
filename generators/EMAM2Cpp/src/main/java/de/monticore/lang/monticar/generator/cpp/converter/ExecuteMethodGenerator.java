/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.MathChainedExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.MathStringExpression;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 * Generates a execute method for math symbols.
 * Can be extended by setSuccessors() to handle unknown symbols. Implements chain-of-responsibility pattern.
 *
 */
public class ExecuteMethodGenerator extends BaseExecuteMethodGeneratorHandler {

    private static ExecuteMethodGenerator ourInstance = new ExecuteMethodGenerator();

    public static ExecuteMethodGenerator getInstance() {
        return ourInstance;
    }

    private ExecuteMethodGenerator() {
    }

    @Override
    public String getRole() {
        return "ExecuteMethodGenerator";
    }

    @Override
    protected boolean canHandleSymbol(MathExpressionSymbol mathExpressionSymbol) {
        boolean canHandle = false;
        if (mathExpressionSymbol == null
                || mathExpressionSymbol.isAssignmentExpression()
                || mathExpressionSymbol.isCompareExpression()
                || mathExpressionSymbol.isForLoopExpression()
                || mathExpressionSymbol.isMatrixExpression()
                || mathExpressionSymbol.isConditionalsExpression()
                || mathExpressionSymbol.isConditionalExpression()
                || mathExpressionSymbol.isArithmeticExpression()
                || mathExpressionSymbol.isValueExpression()
                || mathExpressionSymbol.isMathValueTypeExpression()
                || mathExpressionSymbol.isPreOperatorExpression()
                || (mathExpressionSymbol.getExpressionID() == MathStringExpression.ID)
                || (mathExpressionSymbol.getExpressionID() == MathChainedExpression.ID)
                || mathExpressionSymbol.isParenthesisExpression()) {
            canHandle = true;
        }
        return canHandle;
    }

    protected String doGenerateExecuteCode(MathExpressionSymbol mathExpressionSymbol, List<String> includeStrings) {
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
            Log.debug("ExecuteMethodGenerator", "Case not handled!");

        }
        return result;
    }

    public static String generateExecuteCode(MathExpressionSymbol mathExpressionSymbol, List<String> includeStrings) {
        return getInstance().chainHandleGenerateExecuteCode(mathExpressionSymbol, includeStrings);
    }

    /**
     * Returns correct string for generation of matrix/array access
     *
     * @return
     */
    public static String getCorrectAccessString(String nameOfMathValue, MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol, List<String> includeStrings) {
        String result = "";
        if (ComponentConverterMethodGeneration.currentComponentSymbol.getPortInstance(nameOfMathValue + "[1]").isPresent()) {
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
