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

import de.monticore.lang.monticar.emadl.generator.EMADLGeneratorCli;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.Arrays;

import static junit.framework.TestCase.assertTrue;

public class GenerationTest extends AbstractSymtabTest {

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testCifar10Generation() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "cifar10.Cifar10Classifier", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-emadl"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "cifar10_cifar10Classifier.cpp",
                        "cifar10_cifar10Classifier.h",
                        "CNNCreator_cifar10_cifar10Classifier_net.py",
                        "CNNBufferFile.h",
                        "CNNPredictor_cifar10_cifar10Classifier_net.h",
                        "cifar10_cifar10Classifier_net.h",
                        "CNNTranslator.h",
                        "cifar10_cifar10Classifier_calculateClass.h",
                        "CNNTrainer_cifar10_cifar10Classifier_net.py"));
    }

    @Test
    public void testSimulatorGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "simulator.MainController", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAddGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "Add", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAlexnetGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "Alexnet", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "ResNeXt50", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testThreeInputGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "ThreeInputCNN_M14", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testMultipleOutputsGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "MultipleOutputs", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void tesVGGGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "VGG16", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleInstances() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "InstanceTest.MainB", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMnistClassifier() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "mnist.MnistClassifier", "-b", "CAFFE2"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-emadl"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "mnist_mnistClassifier.cpp",
                        "mnist_mnistClassifier.h",
                        "CNNCreator_mnist_mnistClassifier_net.py",
                        "CNNPredictor_mnist_mnistClassifier_net.h",
                        "mnist_mnistClassifier_net.h",
                        "CNNTranslator.h",
                        "mnist_mnistClassifier_calculateClass.h",
                        "CNNTrainer_mnist_mnistClassifier_net.py"));
    }
}
