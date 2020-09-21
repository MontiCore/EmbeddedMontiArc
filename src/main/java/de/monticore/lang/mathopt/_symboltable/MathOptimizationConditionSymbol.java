/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;

import java.util.List;
import java.util.Optional;

/**
 */
public class MathOptimizationConditionSymbol extends MathExpressionSymbol {

    // fields
    /**
     * optional lower bound of the constraint
     */
    private MathExpressionSymbol lowerBound = null;
    /**
     * optional upper bound of the constraint
     */
    private MathExpressionSymbol upperBound = null;
    /**
     * The expression on which the bounds apply to
     */
    private MathExpressionSymbol boundedExpression;

    // constructors
    public MathOptimizationConditionSymbol(MathExpressionSymbol lower, MathExpressionSymbol expr, MathExpressionSymbol upper) {
        super();
        this.lowerBound = lower;
        this.boundedExpression = expr;
        this.upperBound = upper;
    }

    public MathOptimizationConditionSymbol(MathExpressionSymbol left, String operator, MathExpressionSymbol right) {
        super();
        switch (operator) {
            case "==":
                boundedExpression = left;
                lowerBound = right;
                upperBound = right;
                break;
            case "<=":
                boundedExpression = left;
                upperBound = right;
                break;
            case ">=":
                boundedExpression = left;
                lowerBound = right;
                break;
        }
    }

    // getter
    public Optional<MathExpressionSymbol> getLowerBound() {
        return Optional.ofNullable(lowerBound);
    }

    public Optional<MathExpressionSymbol> getUpperBound() {
        return Optional.ofNullable(upperBound);
    }

    public MathExpressionSymbol getBoundedExpression() {
        return boundedExpression;
    }

    /**
     * Uses optimizationVariable to determine which is the bounded expression and which is the bound and maybe switch
     * bound and bounded expression
     *
     * @param optimizationVariable Optimization Variable of the Optimization Statement
     */
    public void resolveBoundedExpressionToOptimizationVariable(MathValueSymbol optimizationVariable) {
        if (!boundedExpression.getTextualRepresentation().contains(optimizationVariable.getName())) {
            // switch bound(s) and expression
            MathExpressionSymbol newBound = boundedExpression;
            if (getLowerBound().isPresent() && (getUpperBound().isPresent()) && (lowerBound == upperBound)) {
                boundedExpression = lowerBound;
                lowerBound = newBound;
                upperBound = newBound;
            } else if (getLowerBound().isPresent() && !getUpperBound().isPresent()) {
                boundedExpression = lowerBound;
                lowerBound = newBound;
            } else if (getUpperBound().isPresent() && !getLowerBound().isPresent()) {
                boundedExpression = upperBound;
                upperBound = newBound;
            }
        }
    }

    @Override
    public String getTextualRepresentation() {
        String result = boundedExpression.getTextualRepresentation();
        if (getLowerBound().isPresent()) {
            result = String.format("%s <= %s", getLowerBound().get().getTextualRepresentation(), result);
        }
        if (getUpperBound().isPresent()) {
            result = String.format("%s <= %s", result, getUpperBound().get().getTextualRepresentation());
        }
        return result;
    }

    public boolean isMatrixEqualityConstraint(List<MathExpressionSymbol> matrixDimensions) {
        boolean result = false;
        if (getLowerBound().isPresent() && getUpperBound().isPresent()) {
            if (getLowerBound().get().getTextualRepresentation().contentEquals(getUpperBound().get().getTextualRepresentation())) {
                // resolve type
                if (getLowerBound().get() instanceof MathNameExpressionSymbol) {
                    MathNameExpressionSymbol bound = (MathNameExpressionSymbol) getLowerBound().get();
                    MathValueSymbol resolved = (MathValueSymbol) bound.getEnclosingScope().resolve(bound.getNameToResolveValue(), MathValueSymbol.KIND).orElse(null);
                    if ((resolved != null) && (resolved.getType() != null) && (!resolved.getType().getDimensions().isEmpty())) {
                        result = true;
                        matrixDimensions.addAll(resolved.getType().getDimensions());
                    }
                }
            }
        }
        return result;
    }
}
