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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
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

        EMAComponentInstanceSymbol comp = symtab.<EMAComponentInstanceSymbol>resolve("testing.adaptableParameterInstance",EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(comp);

        EMAComponentInstanceSymbol subInst1 = comp.getSubComponent("adaptableParameter").orElse(null);
        assertNotNull(subInst1);

        EMAPortInstanceSymbol configPort = subInst1.getIncomingPortInstance("param1").orElse(null);
        assertNotNull(configPort);
        assertTrue(configPort.isConfig());
        assertTrue(configPort.isIncoming());

        assertNull(subInst1.getIncomingPortInstance("param2").orElse(null));

        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/configPort/");
        List<File> files = generatorCPP.generateFiles(comp, symtab);

        testFilesAreEqual(files,"configPort/");
    }

    @Test
    public void testConfigPortFromKeyword() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol comp = symtab.<EMAComponentInstanceSymbol>resolve("testing.configPort",EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(comp);

        EMAPortInstanceSymbol configPort = comp.getIncomingPortInstance("in1").orElse(null);
        assertNotNull(configPort);
        assertTrue(configPort.isConfig());
        assertTrue(configPort.isIncoming());

        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/configPortFromKeyword/");
        List<File> files = generatorCPP.generateFiles(comp, symtab);

        testFilesAreEqual(files,"configPortFromKeyword/");

    }
}
