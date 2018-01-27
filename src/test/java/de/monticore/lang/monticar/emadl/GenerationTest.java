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

import de.monticore.lang.monticar.emadl.generator.Generator;
import freemarker.template.TemplateException;
import org.junit.Ignore;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class GenerationTest {

    private void generate(String qualifiedName) throws IOException, TemplateException{
        Path modelPath = Paths.get("src/test/resources/");
        Generator gen =  new Generator();
        gen.generate(modelPath, qualifiedName);
    }

    @Ignore
    @Test
    public void testPythonGeneration() throws IOException, TemplateException {
        generate("AlexnetFixedParameters");
    }

    @Ignore
    @Test
    public void testCPPGeneration() throws IOException, TemplateException {
        generate("TargetCPP");
    }
}
