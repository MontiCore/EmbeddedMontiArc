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
package de.monticore.lang.monticar.emadl;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.symboltable.Scope;
import org.junit.Ignore;
import org.junit.Test;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class SymtabTest extends AbstractSymtabTest {

    @Test
    public void testParsing() throws Exception {
        EMADLParser parser = new EMADLParser();
        assertTrue(parser.parse("src/test/resources/Alexnet.emadl").isPresent());
    }

    @Test
    public void testAlexnet(){
        Scope symTab = createSymTab("src/test/resources");
        ComponentSymbol a = symTab.<ComponentSymbol>resolve("Alexnet", ComponentSymbol.KIND).orElse(null);
        ExpandedComponentInstanceSymbol c = symTab.<ExpandedComponentInstanceSymbol>resolve("alexnet", ExpandedComponentInstanceSymbol.KIND).orElse(null);

        /*e.getArchitectureConstructor().resolveArchitecture();
        ArchitectureSymbol arch = e.getArchitectureConstructor().getArchitecture();
        String asds = new CNNArchTemplateController(arch).include(arch.getBody());*/

        //....getComponentType().getReferencedSymbol().getAstNode()
        assertNotNull(a);
    }

    @Ignore
    @Test
    public void test2(){
        Scope symTab = createSymTab("src/test/resources");
        ExpandedComponentInstanceSymbol a = symTab.<ExpandedComponentInstanceSymbol>resolve("mnist", ExpandedComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(a);
    }

}
