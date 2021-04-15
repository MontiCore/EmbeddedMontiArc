/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.opt._ast;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._ast.ASTBehaviorImplementation;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTBehaviorEmbedding;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.math._ast.ASTStatement;
import de.monticore.lang.mathopt._ast.ASTOptimizationStatement;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 */
public class ASTEmbeddedMontiArcMathOptNodesTest {

    @Test
    public void testEMAMOptCompilationUnit() throws IOException {
        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        ASTEMACompilationUnit astNode = parser.parse("src/test/resources/emam-opt/test/emam/optimization/MinimizePortsTest.emam").orElse(null);
        assertNotNull(astNode);
        assertEquals(ASTBehaviorImplementation.class, astNode.getComponent().getBody().getElement(1).getClass());
        ASTBehaviorImplementation behaviour = (ASTBehaviorImplementation) astNode.getComponent().getBody().getElement(1);
        ASTStatement astOptStatement = ((ASTBehaviorEmbedding) behaviour.getBehavior()).getStatement(0);
        assertEquals(ASTOptimizationStatement.class, astOptStatement.getClass());
    }
}
