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
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import static de.monticore.lang.monticar.emadl.ParserTest.ENABLE_FAIL_QUICK;
import static junit.framework.TestCase.assertTrue;
import static de.se_rwth.commons.logging.Log.getFindings;

public class GenerationTest {

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    private void generate(String qualifiedName) throws IOException, TemplateException{
        Generator gen =  new Generator();
        gen.generate("src/test/resources/", qualifiedName);
    }

    @Test
    public void testMnistGeneration() throws IOException, TemplateException {
        generate("mnist.Main");
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testSimulatorGeneration() throws IOException, TemplateException {
        generate("simulator.MainController");
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAddGeneration() throws IOException, TemplateException {
        generate("Add");
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAlexnetGeneration() throws IOException, TemplateException {
        generate("Alexnet");
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        generate("ResNeXt50");
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testThreeInputGeneration() throws IOException, TemplateException {
        generate("ThreeInputCNN_M14");
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testMultipleOutputsGeneration() throws IOException, TemplateException {
        generate("MultipleOutputs");
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void tesVGGGeneration() throws IOException, TemplateException {
        generate("VGG16");
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleInstances() throws IOException, TemplateException {
        generate("InstanceTest.MainB");
        assertTrue(Log.getFindings().isEmpty());
    }
}
