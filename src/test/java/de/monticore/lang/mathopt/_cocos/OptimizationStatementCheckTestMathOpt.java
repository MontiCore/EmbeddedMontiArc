/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._cocos;

import de.monticore.lang.mathopt._ast.ASTOptimizationStatement;
import de.se_rwth.commons.logging.Log;
import org.junit.*;

import java.io.IOException;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 * Tests if OptimizationStatementCheck works correctly
 *
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
        ASTOptimizationStatement ast = getParser().parse_StringOptimizationStatement("minimize Q x; in Q y = x^2; subject to x >= 0; end").orElse(null);
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
        ASTOptimizationStatement ast = getParser().parse_StringOptimizationStatement("minimize Q x; in C y = x^2; subject to x >= 0; end").orElse(null);
        assertNotNull(ast);
        // attach symbol
        initializeSymbol(ast);
        // check
        Log.enableFailQuick(false);
        coco.check(ast);
        assertEquals(1, Log.getErrorCount());
        assertTrue(Log.getFindings().get(0).getMsg().contains("0xC0002"));
    }
}
