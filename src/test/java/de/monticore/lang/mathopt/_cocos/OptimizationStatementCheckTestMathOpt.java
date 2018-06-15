/**
 * ******************************************************************************
 * MontiCAR Modeling Family, www.se-rwth.de
 * Copyright (c) 2018, Software Engineering Group at RWTH Aachen,
 * All rights reserved.
 * <p>
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * <p>
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.mathopt._cocos;

import de.monticore.lang.mathopt._ast.ASTOptimizationStatement;
import de.se_rwth.commons.logging.Log;
import org.junit.After;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.IOException;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 * Tests if OptimizationStatementCheck works correctly
 *
 * @author Christoph Richter
 */
public class OptimizationStatementCheckTestMathOpt extends AbstractMathOptCocoTest {

    private static OptimizationStatementCheck coco;

    @BeforeClass
    public static void setUpClass() throws Exception {
        coco = new OptimizationStatementCheck();
        Log.enableFailQuick(false);
    }

    @Before
    public void setUp() throws Exception {
        Log.getFindings().clear();
    }

    @After
    public void tearDown() throws Exception {
        Log.getFindings().clear();
    }

    @Test
    public void checkValidReturnValue() throws IOException {
        // create ast node
        ASTOptimizationStatement ast = getParser().parse_StringOptimizationStatement("Q y = minimize(Q x) x^2; subject to x >= 0; end").orElse(null);
        assertNotNull(ast);
        // attach symbol
        initializeSymbol(ast);
        // check
        coco.check(ast);
        assertEquals(0, Log.getErrorCount());
    }

    @Test
    public void checkInvalidReturnValue() throws IOException {
        // create ast node
        ASTOptimizationStatement ast = getParser().parse_StringOptimizationStatement("C y = minimize(Q x) x^2; subject to x >= 0; end").orElse(null);
        assertNotNull(ast);
        // attach symbol
        initializeSymbol(ast);
        // check
        Log.enableFailQuick(false);
        coco.check(ast);
        assertEquals(1, Log.getErrorCount());
        assertTrue(Log.getFindings().get(0).getMsg().contains("0x8E75E9"));
    }
}