/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2018, Software Engineering Group at RWTH Aachen,
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
package de.monticore.lang.mathopt._symboltable;

import de.monticore.lang.mathopt.OptimizationModelHelper;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

public class MathOptimizationExpressionSymbolTest {

    // fields
    protected MathOptimizationExpressionSymbol minimizationTestSymbol;
    protected MathOptimizationExpressionSymbol maximizationTestSymbol;
    protected MathOptimizationExpressionSymbol lpTestSymbol;
    protected MathOptimizationExpressionSymbol upperAndLowerBoundTestSymbol;
    protected MathOptimizationExpressionSymbol forLoopConditionTestSymbol;
    protected MathOptimizationExpressionSymbol existingOptimizationVarScalar;
    protected MathOptimizationExpressionSymbol existingOptimizationVarMatrix;
    protected MathOptimizationExpressionSymbol existingOptimizationVarSubstituted;

    // helper
    private OptimizationModelHelper helper;

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
        assertEquals("Q optimization.MinimizationTest.y = x^2", minimizationTestSymbol.getObjectiveExpression().getTextualRepresentation());
    }

    @Test
    public void getSubjectToExpressions() {
        assertTrue(minimizationTestSymbol.getSubjectToExpressions().get(0).getTextualRepresentation().replace(" ", "").contentEquals("x<=1"));
        assertTrue(lpTestSymbol.getSubjectToExpressions().size() == 6);
        assertTrue(upperAndLowerBoundTestSymbol.getSubjectToExpressions().size() == 2);
        assertTrue(forLoopConditionTestSymbol.getSubjectToExpressions().size() >= 1);
    }

}