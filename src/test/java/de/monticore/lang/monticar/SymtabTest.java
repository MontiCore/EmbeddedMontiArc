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
package de.monticore.lang.monticar;

import de.monticore.lang.monticar.stream._symboltable.ComponentStreamSymbol;
import de.monticore.lang.monticar.stream._symboltable.NamedStreamSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.util.Collection;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 * Tests for symbol table of MontiArc.
 *
 * @author Robert Heim
 */
public class SymtabTest extends AbstractSymtabTest {
    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
    }

    @Test
    public void testResolveComponentStreamSymbol() {
        Scope symTab = createSymTab("src/test/resources/nonunitstreams/streams");

        ComponentStreamSymbol comp = symTab.<ComponentStreamSymbol>resolve(
                "basicLibrary.And", ComponentStreamSymbol.KIND).orElse(null);
        assertNotNull(comp);
    }

    @Test
    public void testResolveNamedStreamSymbol() {
        Scope symTab = createSymTab("src/test/resources/nonunitstreams/streams");

        NamedStreamSymbol namedStreamSymbol = symTab.<NamedStreamSymbol>resolve(
                "advancedLibrary.RSFlipFlop.S", NamedStreamSymbol.KIND).orElse(null);
        assertNotNull(namedStreamSymbol);
    }

    @Ignore("ModelPath#resolveModel does not support loading a collection, which should be done when resolving many")
    @Test
    public void testResolveNamedManyStreamSymbol() {
        Scope symTab = createSymTab("src/test/resources/nonunitstreams/streams", "src/test/resources/nonunitstreams/streams2");

        Collection<NamedStreamSymbol> namedStreamSymbols = symTab.<NamedStreamSymbol>resolveMany(
                "advancedLibrary.RSFlipFlop.S", NamedStreamSymbol.KIND);
        assertEquals(2, namedStreamSymbols.size());
    }
}
