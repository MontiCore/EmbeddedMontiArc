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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.opt;

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
 * @author Christoph Richter
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