/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable;

import de.monticore.lang.mathopt.OptimizationModelHelper;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import static org.junit.Assert.*;

public class MathOptimizationStatementSymbolTest {

    // fields
    protected MathOptimizationStatementSymbol minimizationTestSymbol;
    protected MathOptimizationStatementSymbol maximizationTestSymbol;
    protected MathOptimizationStatementSymbol lpTestSymbol;
    protected MathOptimizationStatementSymbol upperAndLowerBoundTestSymbol;
    protected MathOptimizationStatementSymbol forLoopConditionTestSymbol;
    protected MathOptimizationStatementSymbol existingOptimizationVarScalar;
    protected MathOptimizationStatementSymbol existingOptimizationVarMatrix;
    protected MathOptimizationStatementSymbol existingOptimizationVarSubstituted;

    // helper
    private OptimizationModelHelper helper;

    @BeforeClass
    public static void setUpClass() throws Exception {
        Log.enableFailQuick(false);
    }

    @Before
    public void setUp() throws Exception {

        helper = OptimizationModelHelper.getInstance();

        minimizationTestSymbol = helper.getMinimizationTestSymbol();
        maximizationTestSymbol = helper.getMaximizationTestSymbol();
        lpTestSymbol = helper.getLpTestSymbol();
        upperAndLowerBoundTestSymbol = helper.getUpperAndLowerBoundTestSymbol();
        forLoopConditionTestSymbol = helper.getForLoopConditionTestSymbol();
        existingOptimizationVarScalar = helper.getExistingOptimizationVarScalar();
        existingOptimizationVarMatrix = helper.getExistingOptimizationVarMatrix();
        existingOptimizationVarSubstituted = helper.getExistingOptimizationVarSubstituted();
    }

    @Test
    public void getOptimizationType() {
        assertEquals(minimizationTestSymbol.getOptimizationType(), MathOptimizationType.MINIMIZATION);
        assertEquals(maximizationTestSymbol.getOptimizationType(), MathOptimizationType.MAXIMIZATION);
    }

    @Test
    public void getOptimizationVariable() {
        assertTrue(minimizationTestSymbol.getOptimizationVariable().getName().contentEquals("x"));
        assertNotNull(minimizationTestSymbol.getOptimizationVariable().getType());
        assertTrue(existingOptimizationVarScalar.getOptimizationVariable().getName().contentEquals("x"));
        assertNull(existingOptimizationVarScalar.getOptimizationVariable().getType());
        assertTrue(existingOptimizationVarMatrix.getOptimizationVariable().getName().contentEquals("a"));
        assertNull(existingOptimizationVarMatrix.getOptimizationVariable().getType());
        assertTrue(existingOptimizationVarSubstituted.getOptimizationVariable().getName().contentEquals("a"));
        assertNull(existingOptimizationVarSubstituted.getOptimizationVariable().getType());
    }

    @Test
    public void getObjectiveExpression() {
        assertEquals("x^2", minimizationTestSymbol.getObjectiveExpression().getTextualRepresentation());
    }

    @Test
    public void getSubjectToExpressions() {
        assertTrue(minimizationTestSymbol.getSubjectToExpressions().get(0).getTextualRepresentation().replace(" ", "").contentEquals("x<=1"));
        assertTrue(lpTestSymbol.getSubjectToExpressions().size() == 3);
        assertTrue(upperAndLowerBoundTestSymbol.getSubjectToExpressions().size() == 2);
        assertTrue(forLoopConditionTestSymbol.getSubjectToExpressions().size() >= 1);
    }

    @Test
    public void getObjectiveValue() {
        assertEquals(false, minimizationTestSymbol.hasReturnValue());
        assertEquals("y", maximizationTestSymbol.getObjectiveValue().getName());
        assertEquals("Q", maximizationTestSymbol.getObjectiveValue().getType().getType().getName());
    }

}
