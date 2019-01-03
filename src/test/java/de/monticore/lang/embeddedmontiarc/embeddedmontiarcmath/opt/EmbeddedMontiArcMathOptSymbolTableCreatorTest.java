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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.AbstractSymtabTest;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationType;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class EmbeddedMontiArcMathOptSymbolTableCreatorTest extends AbstractSymtabTest {

    @Test
    public void testEMAMMathStatementsSymbol() {
        Scope symTab = createSymTab("src/test/resources/emam-opt/");
        EMAComponentSymbol component = symTab.<EMAComponentSymbol>resolve("test.Add", ComponentSymbol.KIND).orElse(null);
        MathStatementsSymbol statements = (MathStatementsSymbol) component.getSpannedScope().resolve("MathStatements", MathStatementsSymbol.KIND).orElse(null);
        assertNotNull(statements);
        assertEquals(1, statements.getMathExpressionSymbols().size());
        assertEquals(MathAssignmentExpressionSymbol.class, statements.getMathExpressionSymbols().get(0).getClass());
    }

    @Test
    public void testEMAMOptMathStatementsSymbol() {
        Scope symTab = createSymTab("src/test/resources/emam-opt/");
        EMAComponentSymbol component = symTab.<EMAComponentSymbol>resolve("test.math.optimization.ScalarMaximizationTest", ComponentSymbol.KIND).orElse(null);
        MathStatementsSymbol statements = (MathStatementsSymbol) component.getSpannedScope().resolve("MathStatements", MathStatementsSymbol.KIND).orElse(null);
        assertNotNull(statements);
        assertEquals(1, statements.getMathExpressionSymbols().size());
        assertEquals(MathOptimizationStatementSymbol.class, statements.getMathExpressionSymbols().get(0).getClass());
    }

    @Test
    public void testScalarMinimizationModel() {
        Scope symTab = createSymTab("src/test/resources/emam-opt/");
        EMAComponentSymbol component = symTab.<EMAComponentSymbol>resolve("test.math.optimization.ScalarMinimizationTest", ComponentSymbol.KIND).orElse(null);
        MathStatementsSymbol statements = (MathStatementsSymbol) component.getSpannedScope().resolve("MathStatements", MathStatementsSymbol.KIND).orElse(null);
        MathOptimizationStatementSymbol optSymbol = (MathOptimizationStatementSymbol) statements.getMathExpressionSymbols().get(0);
        assertEquals(MathOptimizationType.MINIMIZATION, optSymbol.getOptimizationType());
    }

    @Test
    public void testScalarMaximizationModel() {
        Scope symTab = createSymTab("src/test/resources/emam-opt/");
        EMAComponentSymbol component = symTab.<EMAComponentSymbol>resolve("test.math.optimization.ScalarMaximizationTest", ComponentSymbol.KIND).orElse(null);
        MathStatementsSymbol statements = (MathStatementsSymbol) component.getSpannedScope().resolve("MathStatements", MathStatementsSymbol.KIND).orElse(null);
        MathOptimizationStatementSymbol optSymbol = (MathOptimizationStatementSymbol) statements.getMathExpressionSymbols().get(0);
        assertEquals(MathOptimizationType.MAXIMIZATION, optSymbol.getOptimizationType());
    }
}