/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable;

import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.monticore.lang.mathopt.OptimizationModelHelper;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import static org.junit.Assert.*;

public class MathOptimizationStatementSymbolTest {

    // fields
    protected MathOptimizationStatementSymbol minimizationTestSymbol;
    protected MathOptimizationStatementSymbol maximizationTestSymbol;
    protected MathOptimizationStatementSymbol lpTestSymbol;
    protected MathOptimizationStatementSymbol upperAndLowerBoundTestSymbol;
    protected MathOptimizationStatementSymbol forLoopConditionTestSymbol;
    protected MathOptimizationStatementSymbol mPCTestSymbol;

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
        mPCTestSymbol = helper.getMPCTestSymbol();
    }

    @Test
    public void getOptimizationType() {
        assertEquals(minimizationTestSymbol.getOptimizationType(), MathOptimizationType.MINIMIZATION);
        assertEquals(maximizationTestSymbol.getOptimizationType(), MathOptimizationType.MAXIMIZATION);
    }

    @Test
    public void getOptimizationVariable() {
        assertTrue(minimizationTestSymbol.getOptimizationVariables().get(0).getName().contentEquals("x"));
        assertNotNull(minimizationTestSymbol.getOptimizationVariables().get(0).getType());
        assertTrue(mPCTestSymbol.getOptimizationVariables().get(0).getName().contentEquals("x"));
    }

    @Test
    public void getIndependentVariable(){
        assertTrue(mPCTestSymbol.getIndependentVariables().get(0).getName().contentEquals("y"));
    }

    @Test
    public void getStepSize(){
        MathAssignmentExpressionSymbol maes = (MathAssignmentExpressionSymbol)  mPCTestSymbol.getStepSizeExpression();
        MathMatrixVectorExpressionSymbol mves = (MathMatrixVectorExpressionSymbol) maes.getExpressionSymbol();
        assertTrue(maes.getNameOfMathValue().contentEquals("n"));
        assertTrue(mves.getStart().getTextualRepresentation().equals("1"));
        assertTrue(mves.getEnd().getTextualRepresentation().equals("20"));
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
        assertTrue(mPCTestSymbol.getSubjectToExpressions().size() == 2);
    }

    @Test
    public void getObjectiveValue() {
        assertEquals(false,  minimizationTestSymbol.hasReturnValue());
        assertEquals("y", maximizationTestSymbol.getObjectiveValue().getName());
        assertEquals("Q", maximizationTestSymbol.getObjectiveValue().getType().getType().getName());
        assertEquals("z",mPCTestSymbol.getObjectiveValue().getName());
    }

}
