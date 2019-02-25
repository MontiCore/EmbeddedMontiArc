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

import de.monticore.lang.monticar.emadl.generator.Backend;
import de.monticore.lang.monticar.emadl.generator.EMADLGenerator;
import de.monticore.lang.monticar.emadl.generator.EMADLGeneratorCli;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertFalse;

public class GenerationTest extends AbstractSymtabTest {

    private Path cifarTrainingHashFile = Paths.get("./target/generated-sources-emadl/cifar10/CifarNetwork.training_hash");

    private void createHashFile() {
        try {
            cifarTrainingHashFile.toFile().getParentFile().mkdirs();
            List<String> lines = Arrays.asList("AF9A637D700CB002266D20BF242F4A59#B87F2C80B19CABE0899C30FA66763A47#C4C23549E737A759721D6694C75D9771#5AF0CE68E408E8C1F000E49D72AC214A");
            Files.write(cifarTrainingHashFile, lines, Charset.forName("UTF-8"));
        }
        catch(Exception e) {
            assertFalse("Hash file could not be created", true);
        }
    }

    private void deleteHashFile() {
        try {
            Files.delete(cifarTrainingHashFile);
        }
        catch(Exception e) {
            assertFalse("Could not delete hash file", true);
        }
    }

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testCifar10Generation() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "cifar10.Cifar10Classifier", "-b", "MXNET", "-f", "n"};
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
        String[] args = {"-m", "src/test/resources/models/", "-r", "simulator.MainController", "-b", "MXNET", "-f", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAddGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "Add", "-b", "MXNET", "-f", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAlexnetGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "Alexnet", "-b", "MXNET", "-f", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "ResNeXt50", "-b", "MXNET", "-f", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testThreeInputGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "ThreeInputCNN_M14", "-b", "MXNET", "-f", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testMultipleOutputsGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "MultipleOutputs", "-b", "MXNET", "-f", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testVGGGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "VGG16", "-b", "MXNET", "-f", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleInstances() throws IOException, TemplateException {
        try {
            Log.getFindings().clear();
            String[] args = {"-m", "src/test/resources/models/", "-r", "InstanceTest.MainB", "-b", "MXNET", "-f", "n"};
            EMADLGeneratorCli.main(args);
            assertTrue(Log.getFindings().isEmpty());
        }
        catch(Exception e) {
            e.printStackTrace();
        }
    }

    @Test
    public void testMnistClassifier() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "mnist.MnistClassifier", "-b", "CAFFE2", "-f", "n"};
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

    @Test
    public void testHashFunction() {
        EMADLGenerator tester = new EMADLGenerator(Backend.MXNET);
        
        try{
            tester.getChecksumForFile("invalid Path!");
            assertTrue("Hash method should throw IOException on invalid path", false);
        } catch(IOException e){
        }
    }

    @Test
    public void testDontRetrain1() {
        // The training hash is stored during the first training, so the second one is skipped
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "cifar10.Cifar10Classifier", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
        
        Log.getFindings().clear();
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
        assertTrue(Log.getFindings().get(0).getMsg().contains("skipped"));

        deleteHashFile();
    }

    @Test
    public void testDontRetrain2() {
        // The training hash is written manually, so even the first training should be skipped
        Log.getFindings().clear();
        createHashFile();

        String[] args = {"-m", "src/test/resources/models/", "-r", "cifar10.Cifar10Classifier", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
        assertTrue(Log.getFindings().get(0).getMsg().contains("skipped"));

        deleteHashFile();
    }

    @Test
    public void testDontRetrain3() {
        // Multiple instances of the first NN are used. Only the first one should cause a training
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "instanceTestCifar.MainC", "-b", "MXNET"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
        assertTrue(Log.getFindings().get(0).getMsg().contains("skipped"));
    }

    @Test
    public void testForceRetrain() {
        // The training hash is written manually, but training is forced
        Log.getFindings().clear();
        createHashFile();

        String[] args = {"-m", "src/test/resources/models/", "-r", "cifar10.Cifar10Classifier", "-b", "MXNET", "-f", "y"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        deleteHashFile();
    }
}
