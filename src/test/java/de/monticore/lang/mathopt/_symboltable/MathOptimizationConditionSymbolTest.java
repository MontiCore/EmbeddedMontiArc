/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathForLoopExpressionSymbol;
import de.monticore.lang.mathopt.OptimizationModelHelper;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Unit Tests for MathOptimizationConditionSymbol
 *
 */

public class MathOptimizationConditionSymbolTest {

    // helper
    private OptimizationModelHelper helper;

    // condition symbols
    protected MathOptimizationConditionSymbol minimizationTestConditionSymbol1;
    protected MathOptimizationConditionSymbol lpTestConditionSymbol1;
    protected MathOptimizationConditionSymbol upperAndLowerBoundTestConditionSymbol1;
    protected MathOptimizationConditionSymbol upperAndLowerBoundTestConditionSymbol2;
    protected MathForLoopExpressionSymbol forLoopConditionTestConditionSymbol;

    @Before
    public void setUp() throws Exception {
        helper = OptimizationModelHelper.getInstance();
        minimizationTestConditionSymbol1 = (MathOptimizationConditionSymbol) helper.getMinimizationTestSymbol().getSubjectToExpressions().get(0);
        lpTestConditionSymbol1 = (MathOptimizationConditionSymbol) helper.getLpTestSymbol().getSubjectToExpressions().get(0);
        upperAndLowerBoundTestConditionSymbol1 = (MathOptimizationConditionSymbol) helper.getUpperAndLowerBoundTestSymbol().getSubjectToExpressions().get(0);
        upperAndLowerBoundTestConditionSymbol2 = (MathOptimizationConditionSymbol) helper.getUpperAndLowerBoundTestSymbol().getSubjectToExpressions().get(1);
        forLoopConditionTestConditionSymbol = (MathForLoopExpressionSymbol) helper.getForLoopConditionTestSymbol().getSubjectToExpressions().get(0);
    }

    @Test
    public void getLowerBound() {
        assertFalse(minimizationTestConditionSymbol1.getLowerBound().isPresent());
        assertTrue(lpTestConditionSymbol1.getLowerBound().isPresent());
        assertTrue(upperAndLowerBoundTestConditionSymbol1.getLowerBound().isPresent());
        assertTrue(upperAndLowerBoundTestConditionSymbol2.getLowerBound().isPresent());
    }

    @Test
    public void getUpperBound() {
        assertTrue(minimizationTestConditionSymbol1.getUpperBound().isPresent());
        assertTrue(lpTestConditionSymbol1.getUpperBound().isPresent());
        assertTrue(upperAndLowerBoundTestConditionSymbol1.getUpperBound().isPresent());
        assertFalse(upperAndLowerBoundTestConditionSymbol2.getUpperBound().isPresent());
    }

    @Test
    public void getBoundedExpression() {
        assertTrue(minimizationTestConditionSymbol1.getBoundedExpression().getTextualRepresentation().contains("x"));
        assertTrue(lpTestConditionSymbol1.getBoundedExpression().getTextualRepresentation().contains("x"));
        assertTrue(upperAndLowerBoundTestConditionSymbol1.getBoundedExpression().getTextualRepresentation().contains("x"));
        assertTrue(upperAndLowerBoundTestConditionSymbol2.getBoundedExpression().getTextualRepresentation().contains("x"));
    }

    @Test
    public void testBoundsEqual() {
        MathOptimizationConditionSymbol condition1 = upperAndLowerBoundTestConditionSymbol1;
        assertNotEquals(condition1.getLowerBound().get(), condition1.getUpperBound().get());
        MathOptimizationConditionSymbol condition2 = lpTestConditionSymbol1;
        assertEquals(condition2.getLowerBound().get(), condition2.getUpperBound().get());
    }

    @Test
    public void resolveBoundedExpressionToOptimizationVariable() {
        MathExpressionSymbol bound = minimizationTestConditionSymbol1.getUpperBound().get();
        MathExpressionSymbol expr = minimizationTestConditionSymbol1.getBoundedExpression();
        resolveBoundedExpressionToOptimizationVariableForOperator(bound, expr, "<=");
        resolveBoundedExpressionToOptimizationVariableForOperator(expr, bound, "<=");
        resolveBoundedExpressionToOptimizationVariableForOperator(bound, expr, "==");
        resolveBoundedExpressionToOptimizationVariableForOperator(expr, bound, "==");
        resolveBoundedExpressionToOptimizationVariableForOperator(bound, expr, ">=");
        resolveBoundedExpressionToOptimizationVariableForOperator(expr, bound, ">=");
    }

    private void resolveBoundedExpressionToOptimizationVariableForOperator(MathExpressionSymbol bound, MathExpressionSymbol expr, String op) {
        MathOptimizationConditionSymbol symbol = new MathOptimizationConditionSymbol(bound, op, expr);
        symbol.resolveBoundedExpressionToOptimizationVariable(helper.getMinimizationTestSymbol().getOptimizationVariables());
        assertTrue(symbol.getBoundedExpression().getTextualRepresentation().contains(helper.getMinimizationTestSymbol().getOptimizationVariables().get(0).getName()));
    }


    @Test
    public void testForLoopConditions() {
        assertNotNull(forLoopConditionTestConditionSymbol);
        for (MathExpressionSymbol symbol : forLoopConditionTestConditionSymbol.getForLoopBody()) {
            assertTrue(symbol instanceof MathOptimizationConditionSymbol);
        }
        assertTrue(forLoopConditionTestConditionSymbol.getForLoopBody().size() == 2);
    }
}
