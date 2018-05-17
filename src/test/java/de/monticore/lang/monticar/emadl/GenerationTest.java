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
import de.monticore.lang.monticar.emadl.generator.AbstractSymtab;
import de.monticore.lang.monticar.emadl.generator.EMADLGenerator;
import de.monticore.lang.monticar.emadl.generator.EMADLGeneratorCli;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

import static de.monticore.lang.monticar.emadl.ParserTest.ENABLE_FAIL_QUICK;
import static junit.framework.TestCase.assertEquals;
import static junit.framework.TestCase.assertTrue;

public class GenerationTest extends AbstractSymtabTest {

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    @Test
    public void testCifar10Generation() throws IOException, TemplateException {
        Log.getFindings().clear();;
        String[] args = {"-m", "src/test/resources/models/", "-r", "cifar10.Cifar10Classifier"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-emadl"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "cifar10_cifar10Classifier.h",
                        "CNNCreator_cifar10_cifar10Classifier_net.py",
                        "CNNBufferFile.h",
                        "CNNPredictor_cifar10_cifar10Classifier_net.h",
                        "cifar10_cifar10Classifier_net.h",
                        "CNNTranslator.h",
                        "cifar10_cifar10Classifier_calculateClass.h",
                        "CNNTrainer_cifar10_Cifar10Classifier.py"));
    }

    @Test
    public void testSimulatorGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();;
        String[] args = {"-m", "src/test/resources/models/", "-r", "simulator.MainController"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAddGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();;
        String[] args = {"-m", "src/test/resources/models/", "-r", "Add"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAlexnetGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();;
        String[] args = {"-m", "src/test/resources/models/", "-r", "Alexnet"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();;
        String[] args = {"-m", "src/test/resources/models/", "-r", "ResNeXt50"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testThreeInputGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();;
        String[] args = {"-m", "src/test/resources/models/", "-r", "ThreeInputCNN_M14"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testMultipleOutputsGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();;
        String[] args = {"-m", "src/test/resources/models/", "-r", "MultipleOutputs"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void tesVGGGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();;
        String[] args = {"-m", "src/test/resources/models/", "-r", "VGG16"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleInstances() throws IOException, TemplateException {
        Log.getFindings().clear();;
        String[] args = {"-m", "src/test/resources/models/", "-r", "InstanceTest.MainB"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }
}
