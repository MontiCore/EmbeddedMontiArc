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
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionSourceGenerator;
import de.monticore.lang.monticar.cnnarch.gluongenerator.util.TrainedArchitectureMockFactory;
import de.monticore.lang.monticar.cnntrain.annotations.Range;
import de.monticore.lang.monticar.cnntrain.annotations.TrainedArchitecture;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.List;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import org.junit.contrib.java.lang.system.Assertion;
import org.junit.contrib.java.lang.system.ExpectedSystemExit;
import static junit.framework.TestCase.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class GenerationTest extends AbstractSymtabTest {
    private RewardFunctionSourceGenerator rewardFunctionSourceGenerator;

    @Rule
    public final ExpectedSystemExit exit = ExpectedSystemExit.none();

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
        rewardFunctionSourceGenerator = mock(RewardFunctionSourceGenerator.class);
    }

    @Test
    public void testCifar10Classifier() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "CifarClassifierNetwork", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_CifarClassifierNetwork.py",
                        "CNNNet_CifarClassifierNetwork.py",
                        "CNNDataLoader_CifarClassifierNetwork.py",
                        "CNNSupervisedTrainer_CifarClassifierNetwork.py",
                        "CNNPredictor_CifarClassifierNetwork.h",
                        "execute_CifarClassifierNetwork",
                        "CNNBufferFile.h"));
    }

    @Test
    public void testAlexnetGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "Alexnet", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_Alexnet.py",
                        "CNNNet_Alexnet.py",
                        "CNNDataLoader_Alexnet.py",
                        "CNNSupervisedTrainer_Alexnet.py",
                        "CNNPredictor_Alexnet.h",
                        "execute_Alexnet"));
    }

    @Test
    public void testGeneratorVGG16() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "VGG16", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_VGG16.py",
                        "CNNNet_VGG16.py",
                        "CNNDataLoader_VGG16.py",
                        "CNNSupervisedTrainer_VGG16.py",
                        "CNNPredictor_VGG16.h",
                        "execute_VGG16"));
    }

    @Test
    public void testThreeInputCNNGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ThreeInputCNN_M14"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testInvariant() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "Invariant"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ResNeXt50"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleStreams() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "MultipleStreams"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testFullCfgGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String sourcePath = "src/test/resources/valid_tests";
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        trainGenerator.generate(Paths.get(sourcePath), "FullConfig");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList("CNNTrainer_fullConfig.py"));
    }

    @Test
    public void testSimpleCfgGeneration() throws IOException {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);

        trainGenerator.generate(modelPath, "SimpleConfig");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList("CNNTrainer_simpleConfig.py"));
    }

    @Test
    public void testEmptyCfgGeneration() throws IOException {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        trainGenerator.generate(modelPath, "EmptyConfig");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList("CNNTrainer_emptyConfig.py"));
    }

    @Test
    public void testReinforcementConfig2() {
        // given
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        TrainedArchitecture trainedArchitecture = TrainedArchitectureMockFactory.createTrainedArchitectureMock();

        trainGenerator.generate(modelPath, "ReinforcementConfig2", trainedArchitecture);

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ReinforcementConfig2"),
                Arrays.asList(
                        "CNNTrainer_reinforcementConfig2.py",
                        "start_training.sh",
                        "reinforcement_learning/__init__.py",
                        "reinforcement_learning/strategy.py",
                        "reinforcement_learning/agent.py",
                        "reinforcement_learning/environment.py",
                        "reinforcement_learning/replay_memory.py",
                        "reinforcement_learning/util.py",
                        "reinforcement_learning/cnnarch_logger.py"
                )
        );
    }

    @Test
    public void testReinforcementConfig3() {
        // given
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        TrainedArchitecture trainedArchitecture = TrainedArchitectureMockFactory.createTrainedArchitectureMock();

        trainGenerator.generate(modelPath, "ReinforcementConfig3", trainedArchitecture);

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ReinforcementConfig3"),
                Arrays.asList(
                        "CNNTrainer_reinforcementConfig3.py",
                        "start_training.sh",
                        "reinforcement_learning/__init__.py",
                        "reinforcement_learning/strategy.py",
                        "reinforcement_learning/agent.py",
                        "reinforcement_learning/environment.py",
                        "reinforcement_learning/replay_memory.py",
                        "reinforcement_learning/util.py",
                        "reinforcement_learning/cnnarch_logger.py"
                )
        );
    }

    @Test
    public void testCMakeGeneration() {
        Log.getFindings().clear();
        String rootModelName = "alexnet";
        CNNArch2Gluon generator = new CNNArch2Gluon();
        generator.setGenerationTargetPath("./target/generated-sources-cnnarch");
        generator.generateCMake(rootModelName);

        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList("CMakeLists.txt"));

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch/cmake"),
                Paths.get("./src/test/resources/target_code/cmake"),
                Arrays.asList("FindArmadillo.cmake"));
    }

    @Test
    public void testDdpgConfig() {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests/ddpg");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        TrainedArchitecture trainedArchitecture = TrainedArchitectureMockFactory.createTrainedArchitectureMock();

        trainGenerator.generate(modelPath, "ActorNetwork", trainedArchitecture);

        assertTrue(Log.getFindings().stream().noneMatch(Finding::isError));
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ddpg"),
                Arrays.asList(
                        "CNNTrainer_actorNetwork.py",
                        "start_training.sh",
                        "reinforcement_learning/CNNCreator_CriticNetwork.py",
                        "reinforcement_learning/CNNNet_CriticNetwork.py",
                        "reinforcement_learning/__init__.py",
                        "reinforcement_learning/strategy.py",
                        "reinforcement_learning/agent.py",
                        "reinforcement_learning/environment.py",
                        "reinforcement_learning/replay_memory.py",
                        "reinforcement_learning/util.py",
                        "reinforcement_learning/cnnarch_logger.py"
                )
        );
    }

    @Test
    public void testRosDdpgConfig() {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests/ddpg-ros");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        TrainedArchitecture trainedArchitecture = TrainedArchitectureMockFactory.createTrainedArchitectureMock();

        trainGenerator.generate(modelPath, "RosActorNetwork", trainedArchitecture);

        assertTrue(Log.getFindings().stream().noneMatch(Finding::isError));
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ros-ddpg"),
                Arrays.asList(
                        "CNNTrainer_rosActorNetwork.py",
                        "start_training.sh",
                        "reinforcement_learning/CNNCreator_RosCriticNetwork.py",
                        "reinforcement_learning/CNNNet_RosCriticNetwork.py",
                        "reinforcement_learning/strategy.py",
                        "reinforcement_learning/agent.py",
                        "reinforcement_learning/environment.py",
                        "reinforcement_learning/replay_memory.py",
                        "reinforcement_learning/util.py",
                        "reinforcement_learning/cnnarch_logger.py"
                )
        );
    }

}
