/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._ast;

import de.monticore.lang.mathopt._parser.MathOptParser;
import org.junit.Test;

import java.util.Optional;

import static org.junit.Assert.*;

public class MathOptASTTest {

    @Test
    public void ASTOptimizationStatementTest() throws Exception {
        MathOptParser parser = new MathOptParser();
        Optional<ASTOptimizationStatement> ast = parser.parse_StringOptimizationStatement("minimize Q x; in x; subject to x >= 0; end");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
        assertEquals(ASTOptimizationType.MINIMIZATION, ast.get().optimizationType);
    }

    @Test
    public void ASTOptimizationBoundsConditionTest() throws Exception {
        MathOptParser parser = new MathOptParser();
        Optional<ASTOptimizationBoundsCondition> ast = parser.parse_StringOptimizationBoundsCondition("1 <= x <= 2;");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTOptimizationSimpleConditionTest() throws Exception {
        MathOptParser parser = new MathOptParser();
        Optional<ASTOptimizationSimpleCondition> ast = parser.parse_StringOptimizationSimpleCondition("1 <= x;");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

}
