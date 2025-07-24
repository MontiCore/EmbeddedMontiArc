/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem;

import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Arrays;
import java.util.List;

/**
 * Analyses a MathOptimizationStatementSymbol for its problem class.
 *
 */
public class OptimizationProblemClassification {

    // fields
    private MathOptimizationStatementSymbol symbol;

    private Problem problemType;

    // constructor
    public OptimizationProblemClassification(MathOptimizationStatementSymbol symbol) {
        this.symbol = symbol;
        this.problemType = classifySymbol(symbol);
    }

    /**
     * Classifies MathOptimizationExpression to a optimization problem class
     *
     * @param symbol MathOptimizationStatementSymbol which should be classified
     * @return Optimization problem type
     */
    public static Problem classifySymbol(MathOptimizationStatementSymbol symbol) {

        Problem result = null;
        if (checkIfLP(symbol)) {
            if (isMixedInteger(symbol))
                result = SymbolToProblemConverter.getInstance().getMIPFromSymbol(symbol);
            else
                result = SymbolToProblemConverter.getInstance().getLPFromSymbol(symbol);
        } else if (checkIfQP(symbol)) {
            if (isMixedInteger(symbol))
                result = SymbolToProblemConverter.getInstance().getMIQPFromSymbol(symbol);
            else
                result = SymbolToProblemConverter.getInstance().getQPFromSymbol(symbol);
        } else if (checkIfNLP(symbol)) {
            result = SymbolToProblemConverter.getInstance().getNLPFromSymbol(symbol);
        } else if (checkIfDNLP(symbol)) {
            result = SymbolToProblemConverter.getInstance().getDNLPFromSymbol(symbol);
        } else {
            Log.error(String.format("Can not classify %s: %s", symbol.getClass().toString(), symbol.getFullName()));
        }
        return result;
    }

    private static boolean checkIfQP(MathOptimizationStatementSymbol symbol) {
        boolean result = false;
        //String optvar = symbol.getOptimizationVariable().getName();
        String text = symbol.getTextualRepresentation();
        if (!containsNonLinearFunctions(text)) {

        }
        return result;
    }

    private static boolean containsNonLinearFunctions(String text) {
        List<String> nonLinFunctions = Arrays.asList("sin", "cos", "tan");
        for (String fun : nonLinFunctions) {
            if (text.contains(fun))
                return true;
        }
        return false;
    }

    private static boolean checkIfLP(MathOptimizationStatementSymbol symbol) {
        return false;
    }

    private static boolean checkIfNLP(MathOptimizationStatementSymbol symbol) {
        // if it is not dnlp it must be nlp
        return !checkIfDNLP(symbol);
    }

    private static boolean checkIfDNLP(MathOptimizationStatementSymbol symbol) {
        boolean result = false;
        // if objective function contains branches like if it must be discontius
        MathExpressionSymbol objFunc = symbol.getObjectiveExpression();
        if (objFunc instanceof MathConditionalExpressionsSymbol || objFunc instanceof MathConditionalExpressionSymbol)
            result = true;
        return result;
    }

    private static boolean isMixedInteger(MathOptimizationStatementSymbol symbol) {
        boolean result = false;

        for(MathValueSymbol optimizationVariable : symbol.getOptimizationVariables()){
            MathValueType type = ProblemAssignmentHandler.getInstance().getVariableWithTypeInformations(optimizationVariable).getType();
            if (type.getType().isWholeNumber())
                result = true;
        }

        return result;
    }

    // methods
    public MathOptimizationStatementSymbol getSymbol() {
        return symbol;
    }

    public Problem getProblemType() {
        return problemType;
    }
}
