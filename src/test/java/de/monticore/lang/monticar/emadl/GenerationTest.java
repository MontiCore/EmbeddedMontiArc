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
import de.se_rwth.commons.logging.Finding;
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
import java.util.stream.Collectors;

import static junit.framework.TestCase.assertEquals;
import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertFalse;

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
        String[] args = {"-m", "src/test/resources/models/", "-r", "cifar10.Cifar10Classifier", "-b", "MXNET", "-f", "n", "-c", "n"};
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
        String[] args = {"-m", "src/test/resources/models/", "-r", "simulator.MainController", "-b", "MXNET", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAddGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "Add", "-b", "MXNET", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAlexnetGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "Alexnet", "-b", "MXNET", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "ResNeXt50", "-b", "MXNET", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testThreeInputGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "ThreeInputCNN_M14", "-b", "MXNET", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testMultipleOutputsGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "MultipleOutputs", "-b", "MXNET", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testVGGGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "VGG16", "-b", "MXNET", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleInstances() throws IOException, TemplateException {
        try {
            Log.getFindings().clear();
            String[] args = {"-m", "src/test/resources/models/", "-r", "InstanceTest.MainB", "-b", "MXNET", "-f", "n", "-c", "n"};
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
        String[] args = {"-m", "src/test/resources/models/", "-r", "mnist.MnistClassifier", "-b", "CAFFE2", "-f", "n", "-c", "n"};
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
    public void testMnistClassifierForGluon() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "mnist.MnistClassifier", "-b", "GLUON", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-emadl"),
                Paths.get("./src/test/resources/target_code/gluon"),
                Arrays.asList(
                        "CNNBufferFile.h",
                        "CNNNet_mnist_mnistClassifier_net.py",
                        "mnist_mnistClassifier.cpp",
                        "mnist_mnistClassifier.h",
                        "CNNCreator_mnist_mnistClassifier_net.py",
                        "CNNPredictor_mnist_mnistClassifier_net.h",
                        "CNNDataLoader_mnist_mnistClassifier_net.py",
                        "supervised_trainer.py",
                        "mnist_mnistClassifier_net.h",
                        "HelperA.h",
                        "CNNTranslator.h",
                        "mnist_mnistClassifier_calculateClass.h",
                        "CNNTrainer_mnist_mnistClassifier_net.py",
                        "mnist_mnistClassifier_net.h"));
    }

    @Test
    public void testGluonReinforcementModelGymEnvironment() {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/reinforcementModel", "-r", "cartpole.Master", "-b", "GLUON", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().stream().filter(Finding::isError).collect(Collectors.toList()).isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-emadl"),
                Paths.get("./src/test/resources/target_code/gluon/reinforcementModel/cartpole"),
                Arrays.asList(
                        "cartpole_master.cpp",
                        "cartpole_master.h",
                        "cartpole_master_dqn.h",
                        "cartpole_master_policy.h",
                        "CMakeLists.txt",
                        "CNNBufferFile.h",
                        "CNNCreator_cartpole_master_dqn.py",
                        "CNNNet_cartpole_master_dqn.py",
                        "CNNPredictor_cartpole_master_dqn.h",
                        "CNNTrainer_cartpole_master_dqn.py",
                        "CNNTranslator.h",
                        "HelperA.h",
                        "start_training.sh",
                        "reinforcement_learning/__init__.py",
                        "reinforcement_learning/action_policy.py",
                        "reinforcement_learning/agent.py",
                        "reinforcement_learning/environment.py",
                        "reinforcement_learning/replay_memory.py",
                        "reinforcement_learning/util.py"
                )
        );
    }

    @Test
    public void testGluonReinforcementModelRosEnvironment() {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/reinforcementModel", "-r", "torcs.agent.TorcsAgent", "-b", "GLUON", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().stream().filter(Finding::isError).collect(Collectors.toList()).isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-emadl"),
                Paths.get("./src/test/resources/target_code/gluon/reinforcementModel/torcs"),
                Arrays.asList(
                        "CMakeLists.txt",
                        "CNNBufferFile.h",
                        "torcs_agent_torcsAgent.cpp",
                        "torcs_agent_torcsAgent.h",
                        "torcs_agent_torcsAgent_dqn.h",
                        "torcs_agent_torcsAgent_policy.h",
                        "CNNCreator_torcs_agent_torcsAgent_dqn.py",
                        "CNNNet_torcs_agent_torcsAgent_dqn.py",
                        "CNNPredictor_torcs_agent_torcsAgent_dqn.h",
                        "CNNTrainer_torcs_agent_torcsAgent_dqn.py",
                        "CNNTranslator.h",
                        "HelperA.h",
                        "start_training.sh",
                        "reward/CMakeLists.txt",
                        "reward/HelperA.h",
                        "reward/torcs_agent_dqn_reward.cpp",
                        "reward/torcs_agent_dqn_reward.h",
                        "reward/pylib/CMakeLists.txt",
                        "reward/pylib/torcs_agent_dqn_reward_executor.cpp",
                        "reward/pylib/torcs_agent_dqn_reward_executor.h",
                        "reward/pylib/torcs_agent_dqn_reward_executor.i",
                        "reward/pylib/armanpy/armanpy.hpp",
                        "reward/pylib/armanpy/armanpy.i",
                        "reward/pylib/armanpy/armanpy_1d.i",
                        "reward/pylib/armanpy/armanpy_2d.i",
                        "reward/pylib/armanpy/armanpy_3d.i",
                        "reward/pylib/armanpy/numpy.i",
                        "reinforcement_learning/__init__.py",
                        "reinforcement_learning/action_policy.py",
                        "reinforcement_learning/agent.py",
                        "reinforcement_learning/environment.py",
                        "reinforcement_learning/replay_memory.py",
                        "reinforcement_learning/util.py",
                        "reinforcement_learning/torcs_agent_dqn_reward_executor.py"
                )
        );
        assertTrue(Paths.get(
                "./target/generated-sources-emadl/reinforcement_learning/_torcs_agent_dqn_reward_executor.so")
                .toFile().exists());
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
    public void gluonDdpgTest() {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/reinforcementModel", "-r", "mountaincar.Master", "-b", "GLUON", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertEquals(0, Log.getFindings().stream().filter(Finding::isError).count());
    }
}
