/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 */

public class MathOptimizationStatementSymbol extends MathExpressionSymbol {
    //region fields

    /**
     * Defines if minimization or maximization is performed
     */
    private MathOptimizationType optimizationType;
    /**
     * Variable which will be minimized/ maximized
     */
    private List<MathValueSymbol> optimizationVariables = new ArrayList<>();;

    private List<MathValueSymbol> independentVariables = new ArrayList<>();;
    /**
     * Variable which will store the result of the optimization
     */
    private MathValueSymbol objectiveValue;
    /**
     * The expression which should be minimized / maximized
     */
    private MathExpressionSymbol objectiveExpression;
    /**
     * List of 0..n constraints
     */
    private List<MathExpressionSymbol> subjectToExpressions = new ArrayList<>();

    /**
     * List of all constraint expressions, unmodified.
     */
    private List<MathOptimizationConditionSymbol> constraints = new ArrayList<>();

    private MathExpressionSymbol stepSizeExpression;

    //endregion
    // region constructor


    public MathOptimizationStatementSymbol() {
        super();
    }

    public MathOptimizationStatementSymbol(String name) {
        super(name);
    }

    // endregion
    // region getter setter methods
    public MathOptimizationType getOptimizationType() {
        return optimizationType;
    }

    public void setOptimizationType(String optimizationTypeString) {
        if (optimizationTypeString.toLowerCase().contains("min")) {
            this.optimizationType = MathOptimizationType.MINIMIZATION;
        } else if (optimizationTypeString.toLowerCase().contains("max")) {
            this.optimizationType = MathOptimizationType.MAXIMIZATION;
        }
    }

    public List<MathValueSymbol> getOptimizationVariables() {
        return optimizationVariables;
    }

    public void setOptimizationVariables(List<MathValueSymbol> optimizationVariables) {
        this.optimizationVariables = optimizationVariables;
    }

    public List<MathValueSymbol> getIndependentVariables() {
        return independentVariables;
    }

    public void setIndependentVariables(List<MathValueSymbol> independentVariables) {
        this.independentVariables = independentVariables;
    }

    public MathExpressionSymbol getObjectiveExpression() {
        return objectiveExpression;
    }

    public void setObjectiveExpression(MathExpressionSymbol objectiveExpression) {
        this.objectiveExpression = objectiveExpression;
    }

    public List<MathExpressionSymbol> getSubjectToExpressions() {
        return subjectToExpressions;
    }

    public MathValueSymbol getObjectiveValue() {
        return objectiveValue;
    }

    public void setObjectiveValue(MathValueSymbol objectiveValue) {
        this.objectiveValue = objectiveValue;
    }

    public boolean hasReturnValue() {
        return objectiveValue != null;
    }

    public MathExpressionSymbol getStepSizeExpression() {
        return stepSizeExpression;
    }

    public void setStepSizeExpression(MathExpressionSymbol stepSizeExpression) {
        this.stepSizeExpression = stepSizeExpression;
    }

    public List<MathOptimizationConditionSymbol> getConstraints() {
        return constraints;
    }

    public void setConstraints(List<MathOptimizationConditionSymbol> constraints) {
        this.constraints = constraints;
    }

    // endregion
    // region methods
    @Override
    public String getTextualRepresentation() {

        StringBuilder result = new StringBuilder(String.format("minimize/n"));
        for (int i=0; i < optimizationVariables.size()-1;i++){
            result.append("  "+optimizationVariables.get(i) + ",\n");
        }
        if(!optimizationVariables.isEmpty()) {
            result.append("  " + optimizationVariables.get(optimizationVariables.size() - 1) + ";\n");
        }
        result.append("in");
        result.append(String.format("  %s /n", objectiveExpression.getTextualRepresentation()));
        result.append("subject to \n");
        for (MathExpressionSymbol symbol : subjectToExpressions) {
            result.append(symbol.getTextualRepresentation()).append(";\n");
        }
        result.append("end");
        return result.toString();
    }
    // endregion
}
