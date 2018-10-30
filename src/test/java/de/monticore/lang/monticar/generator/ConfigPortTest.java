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
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class ConfigPortTest extends AbstractSymtabTest{

    @Test
    public void testConfigPortFromAdaptableParameter() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol comp = symtab.<ExpandedComponentInstanceSymbol>resolve("testing.adaptableParameterInstance",ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(comp);

        ExpandedComponentInstanceSymbol subInst1 = comp.getSubComponent("adaptableParameter").orElse(null);
        assertNotNull(subInst1);

        PortSymbol configPort = subInst1.getIncomingPort("param1").orElse(null);
        assertNotNull(configPort);
        assertTrue(configPort.isConfig());
        assertTrue(configPort.isIncoming());

        assertNull(subInst1.getIncomingPort("param2").orElse(null));

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/configPort/");
        List<File> files = generatorCPP.generateFiles(comp, symtab);

        testFilesAreEqual(files,"configPort/");
    }

    @Test
    public void testConfigPortFromKeyword() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol comp = symtab.<ExpandedComponentInstanceSymbol>resolve("testing.configPort",ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(comp);

        PortSymbol configPort = comp.getIncomingPort("in1").orElse(null);
        assertNotNull(configPort);
        assertTrue(configPort.isConfig());
        assertTrue(configPort.isIncoming());

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/configPortFromKeyword/");
        List<File> files = generatorCPP.generateFiles(comp, symtab);

        testFilesAreEqual(files,"configPortFromKeyword/");

    }
}
