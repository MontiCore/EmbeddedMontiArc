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

import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionSourceGenerator;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import static junit.framework.TestCase.assertTrue;
import static org.mockito.Mockito.mock;

public class GenerationTest extends AbstractSymtabTest {
    private RewardFunctionSourceGenerator rewardFunctionSourceGenerator;

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
                        "CNNPredictor_VGG16.h",
                        "execute_VGG16"));
    }


    @Test
    public void testThreeInputCNNGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ThreeInputCNN_M14"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ResNeXt50"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleOutputs() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "MultipleOutputs"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().size() == 1);
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
                Arrays.asList(
                        "CNNTrainer_fullConfig.py",
                        "supervised_trainer.py"));
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
                Arrays.asList(
                        "CNNTrainer_simpleConfig.py",
                        "supervised_trainer.py"));
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
                Arrays.asList(
                        "CNNTrainer_emptyConfig.py",
                        "supervised_trainer.py"));
    }

    @Test
    public void testReinforcementConfig2() {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);

        trainGenerator.generate(modelPath, "ReinforcementConfig2");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ReinforcementConfig2"),
                Arrays.asList(
                        "CNNTrainer_reinforcementConfig2.py",
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
                Arrays.asList(
                        "CMakeLists.txt"));

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch/cmake"),
                Paths.get("./src/test/resources/target_code/cmake"),
                Arrays.asList(
                        "FindArmadillo.cmake"));
    }

}
