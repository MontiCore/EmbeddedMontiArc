/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.math.math._symboltable;

import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathValueSymbol;

import java.util.Optional;

/**
 * @author Christoph Richter
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
}
