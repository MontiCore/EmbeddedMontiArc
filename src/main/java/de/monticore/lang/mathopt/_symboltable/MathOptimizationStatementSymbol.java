/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Christoph Richter
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
    private MathValueSymbol optimizationVariable;
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

    public MathValueSymbol getOptimizationVariable() {
        return optimizationVariable;
    }

    public void setOptimizationVariable(MathValueSymbol optimizationVariable) {
        this.optimizationVariable = optimizationVariable;
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

    // endregion
    // region methods
    @Override
    public String getTextualRepresentation() {
        StringBuilder result = new StringBuilder(String.format("minimize(%s) /n", optimizationVariable.getTextualRepresentation()));
        result.append(String.format("%s /n", objectiveExpression.getTextualRepresentation()));
        result.append("subject to \n");
        for (MathExpressionSymbol symbol : subjectToExpressions) {
            result.append(symbol.getTextualRepresentation()).append(";\n");
        }
        result.append("end");
        return result.toString();
    }
    // endregion
}
