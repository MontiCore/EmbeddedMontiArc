/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.gluongenerator.AbstractSymtabTest;
import de.monticore.lang.monticar.cnnarch.gluongenerator.CNNArch2Gluon;
import de.monticore.lang.monticar.cnnarch.gluongenerator.CNNArch2GluonCli;
import de.monticore.lang.monticar.cnnarch.gluongenerator.CNNTrain2Gluon;
import de.monticore.lang.monticar.cnnarch.generator.GenerationAbortedException;
import de.monticore.lang.monticar.cnnarch.generator.annotations.ArchitectureAdapter;
import de.monticore.lang.monticar.cnnarch.generator.reinforcement.RewardFunctionSourceGenerator;
import de.monticore.lang.monticar.cnnarch.gluongenerator.util.NNArchitectureMockFactory;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.contrib.java.lang.system.ExpectedSystemExit;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

import static junit.framework.Assert.assertEquals;
import static junit.framework.Assert.fail;
import static junit.framework.TestCase.assertTrue;
import static org.mockito.Mockito.mock;

public class GenerationTest extends AbstractSymtabTest {
    private RewardFunctionSourceGenerator rewardFunctionSourceGenerator;

    @Rule
    public final ExpectedSystemExit exit = ExpectedSystemExit.none();

    @Before
    public void setUp() {
        // ensure an empty log
        Log.initWARN();
        Log.getFindings().clear();
        Log.enableFailQuick(false);
        rewardFunctionSourceGenerator = mock(RewardFunctionSourceGenerator.class);
    }

    @Test
    public void testCifar10Classifier() throws IOException, TemplateException {
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "CifarClassifierNetwork", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("target/generated-sources-cnnarch"),
                Paths.get("src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_CifarClassifierNetwork.py",
                        "CNNDatasets_CifarClassifierNetwork.py",
                        "CNNNet_CifarClassifierNetwork.py",
                        "CNNDataLoader_CifarClassifierNetwork.py",
                        "CNNSupervisedTrainer_CifarClassifierNetwork.py",
                        "CNNPredictor_CifarClassifierNetwork.h",
                        "execute_CifarClassifierNetwork",
                        "CNNModelLoader.h"));
    }

    @Test
    public void testAlexnetGeneration() {
        String[] args = {"-m", "src/test/resources/architectures", "-r", "Alexnet", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_Alexnet.py",
                        "CNNDatasets_Alexnet.py",
                        "CNNNet_Alexnet.py",
                        "CNNDataLoader_Alexnet.py",
                        "CNNSupervisedTrainer_Alexnet.py",
                        "CNNPredictor_Alexnet.h",
                        "execute_Alexnet",
                        "CNNModelLoader.h"));
    }

    @Test
    public void testGeneratorVGG16() throws IOException, TemplateException {
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
                        "execute_VGG16",
                        "CNNModelLoader.h"));
    }

    @Test
    public void testThreeInputCNNGeneration() throws IOException, TemplateException {
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ThreeInputCNN_M14"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testInvariant() throws IOException, TemplateException {
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "Invariant"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ResNeXt50"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleStreams() throws IOException, TemplateException {
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "MultipleStreams"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testRNNtest() throws IOException, TemplateException {
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "RNNtest", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testRNNencdec() throws IOException, TemplateException {
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "RNNencdec", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testRNNsearch() throws IOException, TemplateException {
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "RNNsearch", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testShow_attend_tell() throws IOException, TemplateException {
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "Show_attend_tell", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testEpisodicMemoryGeneration() throws IOException, TemplateException {
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "EpisodicMemoryNetwork", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_EpisodicMemoryNetwork.py",
                        "CNNDatasets_EpisodicMemoryNetwork.py",
                        "CNNNet_EpisodicMemoryNetwork.py",
                        "CNNDataLoader_EpisodicMemoryNetwork.py",
                        "CNNSupervisedTrainer_EpisodicMemoryNetwork.py",
                        "CNNPredictor_EpisodicMemoryNetwork.h",
                        "execute_EpisodicMemoryNetwork",
                        "CNNModelLoader.h"));
    }

    @Test
    public void testLoadNetworkLayerGeneration() throws IOException, TemplateException {
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "LoadNetworkTest", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_LoadNetworkTest.py",
                        "CNNDatasets_LoadNetworkTest.py",
                        "CNNNet_LoadNetworkTest.py",
                        "CNNDataLoader_LoadNetworkTest.py",
                        "CNNSupervisedTrainer_LoadNetworkTest.py",
                        "CNNPredictor_LoadNetworkTest.h",
                        "execute_LoadNetworkTest",
                        "CNNModelLoader.h"));
    }

    @Test
    public void testFullCfgGeneration() throws IOException, TemplateException {
        String sourcePath = "src/test/resources/valid_tests";
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        trainGenerator.generate(Paths.get(sourcePath), "FullConfig");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNTrainer_fullConfig.py",
                        "CNNLAOptimizer_fullConfig.h"));
    }

    @Test
    public void testSimpleCfgGeneration() throws IOException {
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);

        trainGenerator.generate(modelPath, "SimpleConfig");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNTrainer_simpleConfig.py",
                        "CNNLAOptimizer_simpleConfig.h"));
    }

    @Test
    public void testEmptyCfgGeneration() throws IOException {
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        trainGenerator.generate(modelPath, "EmptyConfig");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNTrainer_emptyConfig.py",
                        "CNNLAOptimizer_emptyConfig.h"));
    }

    @Test
    public void testReinforcementConfig2() {
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter trainedArchitecture = NNArchitectureMockFactory.createDQNMock();

        trainGenerator.generate(modelPath, "ReinforcementConfig2", trainedArchitecture);

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ReinforcementConfig2"),
                Arrays.asList(
                        "CNNTrainer_reinforcementConfig2.py",
                        "CNNLAOptimizer_reinforcementConfig2.h",
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
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter trainedArchitecture = NNArchitectureMockFactory.createDQNMock();

        trainGenerator.generate(modelPath, "ReinforcementConfig3", trainedArchitecture);

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ReinforcementConfig3"),
                Arrays.asList(
                        "CNNTrainer_reinforcementConfig3.py",
                        "CNNLAOptimizer_reinforcementConfig3.h",
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
    public void testReinforcementConfig4() {
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter trainedArchitecture = NNArchitectureMockFactory.createDQNDiscreteMock();

        trainGenerator.generate(modelPath, "ReinforcementConfig4", trainedArchitecture);

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ReinforcementConfig4"),
                Arrays.asList(
                        "CNNTrainer_reinforcementConfig4.py",
                        "CNNLAOptimizer_reinforcementConfig4.h",
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
        Path modelPath = Paths.get("src/test/resources/valid_tests/ddpg");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();
        ArchitectureAdapter criticArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/ddpg/comp"), "CriticNetwork");

        trainGenerator.generate(modelPath, "ActorNetwork", trainedArchitecture, criticArchitecture);

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ddpg"),
                Arrays.asList(
                        "CNNTrainer_actorNetwork.py",
                        "CNNLAOptimizer_actorNetwork.h",
                        "start_training.sh",
                        "reinforcement_learning/CNNCreator_CriticNetwork.py",
                        "reinforcement_learning/CNNDatasets_CriticNetwork.py",
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
    public void testTd3Config() {
        Path modelPath = Paths.get("src/test/resources/valid_tests/td3");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();
        ArchitectureAdapter criticArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/td3/comp"), "CriticNetwork");

        trainGenerator.generate(modelPath, "TD3Config", trainedArchitecture, criticArchitecture);

        assertTrue(Log.getFindings().stream().noneMatch(Finding::isError));
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/td3"),
                Arrays.asList(
                        "CNNTrainer_tD3Config.py",
                        "CNNLAOptimizer_tD3Config.h",
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
        Path modelPath = Paths.get("src/test/resources/valid_tests/ddpg-ros");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();
        ArchitectureAdapter criticArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/ddpg-ros/comp"), "RosCriticNetwork");

        trainGenerator.generate(modelPath, "RosActorNetwork", trainedArchitecture, criticArchitecture);

        assertEquals(0, Log.getErrorCount());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ros-ddpg"),
                Arrays.asList(
                        "CNNTrainer_rosActorNetwork.py",
                        "CNNLAOptimizer_rosActorNetwork.h",
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

    @Test
    public void testDefaultGANConfig() {
        Path modelPath = Paths.get("src/test/resources/valid_tests/default-gan");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter genArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/default-gan/arc"), "DefaultGAN");
        ArchitectureAdapter disArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/default-gan/arc"), "Discriminator");

        trainGenerator.generate(modelPath, "DefaultGAN", genArchitecture, disArchitecture, null);

        assertTrue(Log.getFindings().stream().noneMatch(Finding::isError));
        checkFilesAreEqual(
                Paths.get("target/generated-sources-cnnarch"),
                Paths.get("src/test/resources/target_code/default-gan"),
                Arrays.asList(
                        "gan/CNNCreator_Discriminator.py",
                        "gan/CNNNet_Discriminator.py",
                        "CNNTrainer_defaultGAN.py",
                        "CNNLAOptimizer_defaultGAN.h"
                )
        );
    }

    @Test
    public void testInfoGANConfig() {
        Path modelPath = Paths.get("src/test/resources/valid_tests/info-gan");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter genArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/info-gan/arc"), "InfoGAN");
        ArchitectureAdapter disArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/info-gan/arc"), "InfoDiscriminator");
        ArchitectureAdapter qnetArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/info-gan/arc"), "InfoQNetwork");

        trainGenerator.generate(modelPath, "InfoGAN", genArchitecture, disArchitecture, qnetArchitecture);

        assertTrue(Log.getFindings().stream().noneMatch(Finding::isError));
        checkFilesAreEqual(
                Paths.get("target/generated-sources-cnnarch"),
                Paths.get("src/test/resources/target_code/info-gan"),
                Arrays.asList(
                        "gan/CNNCreator_InfoDiscriminator.py",
                        "gan/CNNNet_InfoDiscriminator.py",
                        "gan/CNNCreator_InfoQNetwork.py",
                        "gan/CNNNet_InfoQNetwork.py",
                        "CNNTrainer_infoGAN.py",
                        "CNNLAOptimizer_infoGAN.h"
                )
        );
    }


    @Test
    public void testVAETestNetGeneration() throws IOException, TemplateException {

        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests/vae");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter encoderArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/vae/arc"), "Encoder");
        ArchitectureAdapter decoderArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/vae/arc"), "Decoder");

        trainGenerator.generate(modelPath, "Decoder", decoderArchitecture, encoderArchitecture);

        assertTrue(Log.getFindings().stream().noneMatch(Finding::isError));
        checkFilesAreEqual(
                Paths.get("target/generated-sources-cnnarch"),
                Paths.get("src/test/resources/target_code/vae"),
                Arrays.asList(
                        "CNNAutoencoderTrainer_Encoder.py",
                        "CNNCreator_Encoder.py",
                        "CNNNet_Encoder.py",
                        "CNNTrainer_decoder.py"));

    }

    @Test
    public void testVQVAETestNetGeneration() throws IOException, TemplateException {

        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests/vqvae");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter encoderArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/vqvae/arc"), "VQEncoder");
        ArchitectureAdapter decoderArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/vqvae/arc"), "VQDecoder");

        trainGenerator.generate(modelPath, "VQDecoder", decoderArchitecture, encoderArchitecture);

        assertTrue(Log.getFindings().stream().noneMatch(Finding::isError));
        checkFilesAreEqual(
                Paths.get("target/generated-sources-cnnarch"),
                Paths.get("src/test/resources/target_code/vqvae"),
                Arrays.asList(
                        "CNNAutoencoderTrainer_VQEncoder.py",
                        "CNNCreator_VQEncoder.py",
                        "CNNNet_VQEncoder.py",
                        "CNNTrainer_vQDecoder.py"));


    }


    @Test
    public void testGenerationWithoutTrainingConfigurationFails() {
        Path modelPath = Paths.get("src/test/resources/valid_tests");

        try {
            CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
            trainGenerator.generate(modelPath, "ModelWithoutTrainingConfiguration");
            fail("A RuntimeException should have been thrown!");
        } catch (RuntimeException e) {
            assertEquals(1, Log.getErrorCount());
            assertEquals("Could not resolve training configuration for model 'ModelWithoutTrainingConfiguration'.", e.getMessage());
        }
    }

    @Test
    public void testSchemaIsCheckedBeforeGenerationStarts() {
        Path modelPath = Paths.get("src/test/resources/valid_tests");

        try {
            CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
            trainGenerator.generate(modelPath, "InvalidSchemaDefinition");
            fail("A GenerationAbortedException should have been thrown!");
        } catch (GenerationAbortedException e) {
            assertEquals("Generation aborted due to errors in the training configuration.", e.getMessage());
        }
    }

    @Test
    public void checkReinforcementRequiresEnvironment() {
        Path modelPath = Paths.get("src/test/resources/valid_tests/model-checks/reinforcement/ddpg");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();
        ArchitectureAdapter criticArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/model-checks/reinforcement/ddpg/comp"), "CriticNetwork");

        try {
            trainGenerator.generate(modelPath, "ActorNetwork_No_Environment", trainedArchitecture, criticArchitecture);
            fail("A GenerationAbortedException should have been thrown!");
        } catch (GenerationAbortedException e) {
            assertEquals(1, Log.getErrorCount());
            assertEquals("Generation aborted due to errors in the training configuration.", e.getMessage());
            assertTrue(containsErrorWithMessage(Log.getFindings(), "0xSL16C Required parameter 'environment' is missing."));
        }
    }

    @Test
    public void testDdpg_InvalidStrategyDefined() {
        Path modelPath = Paths.get("src/test/resources/valid_tests/model-checks/reinforcement/ddpg");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();
        ArchitectureAdapter criticArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/model-checks/reinforcement/ddpg/comp"), "CriticNetwork");

        try {
            trainGenerator.generate(modelPath, "ActorNetwork_Invalid_Strategy", trainedArchitecture, criticArchitecture);
            fail("A GenerationAbortedException should have been thrown!");
        } catch (GenerationAbortedException e) {
            assertEquals(1, Log.getErrorCount());
            assertEquals("Generation aborted due to errors in the training configuration.", e.getMessage());
            assertTrue(containsErrorWithMessage(Log.getFindings(), "0xSL12C Value 'epsgreedy' is not applicable for parameter 'strategy'. The following values are allowed: ornstein_uhlenbeck, gaussian"));
        }
    }

    @Test
    public void testTd3_InvalidStrategyDefined() {
        Path modelPath = Paths.get("src/test/resources/valid_tests/model-checks/reinforcement/td3");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();
        ArchitectureAdapter criticArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/model-checks/reinforcement/td3/comp"), "CriticNetwork");

        try {
            trainGenerator.generate(modelPath, "TD3Config_Invalid_Strategy", trainedArchitecture, criticArchitecture);
            fail("A GenerationAbortedException should have been thrown!");
        } catch (GenerationAbortedException e) {
            assertEquals(1, Log.getErrorCount());
            assertEquals("Generation aborted due to errors in the training configuration.", e.getMessage());
            assertTrue(containsErrorWithMessage(Log.getFindings(), "0xSL12C Value 'epsgreedy' is not applicable for parameter 'strategy'. The following values are allowed: ornstein_uhlenbeck, gaussian"));
        }
    }

    @Test
    public void testDefaultGAN_NoNoiseDistributionDefined() {
        Path modelPath = Paths.get("src/test/resources/valid_tests/model-checks/gan/default-gan");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter genArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/model-checks/gan/default-gan/arc"), "DefaultGAN");
        ArchitectureAdapter disArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/model-checks/gan/default-gan/arc"), "Discriminator");

        try {
            trainGenerator.generate(modelPath, "DefaultGAN_NoNoiseDistribution", genArchitecture, disArchitecture, null);
            fail("A GenerationAbortedException should have been thrown!");
        } catch (GenerationAbortedException e) {
            assertEquals(1, Log.getErrorCount());
            assertEquals("Generation aborted due to errors in the training configuration.", e.getMessage());
            assertTrue(containsErrorWithMessage(Log.getFindings(), "0xSL21C Setting parameter 'noise_input' requires also setting parameter 'noise_distribution'."));
        }
    }

    @Test
    public void testDefaultGAN_NoGeneratorTargetName() {
        Path modelPath = Paths.get("src/test/resources/valid_tests/model-checks/gan/default-gan");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter genArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/model-checks/gan/default-gan/arc"), "DefaultGAN");
        ArchitectureAdapter disArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/model-checks/gan/default-gan/arc"), "Discriminator");

        try {
            trainGenerator.generate(modelPath, "DefaultGAN_NoGeneratorTargetName", genArchitecture, disArchitecture, null);
            fail("A GenerationAbortedException should have been thrown!");
        } catch (GenerationAbortedException e) {
            assertEquals(1, Log.getErrorCount());
            assertEquals("Generation aborted due to errors in the training configuration.", e.getMessage());
            assertTrue(containsErrorWithMessage(Log.getFindings(), "0xSL21C Setting parameter 'generator_loss' requires also setting parameter 'generator_target_name'."));
        }
    }

    @Test
    public void testDefaultGAN_NoGeneratorLoss() {
        Path modelPath = Paths.get("src/test/resources/valid_tests/model-checks/gan/default-gan");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter genArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/model-checks/gan/default-gan/arc"), "DefaultGAN");
        ArchitectureAdapter disArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/model-checks/gan/default-gan/arc"), "Discriminator");

        try {
            trainGenerator.generate(modelPath, "DefaultGAN_NoGeneratorLoss", genArchitecture, disArchitecture, null);
            fail("A GenerationAbortedException should have been thrown!");
        } catch (GenerationAbortedException e) {
            assertEquals(1, Log.getErrorCount());
            assertEquals("Generation aborted due to errors in the training configuration.", e.getMessage());
            assertTrue(containsErrorWithMessage(Log.getFindings(), "0xSL21C Setting parameter 'generator_target_name' requires also setting parameter 'generator_loss'."));
        }
    }

    @Test
    public void checkRosEnvironmentRequiresRewardFunction() {
        Path modelPath = Paths.get("src/test/resources/valid_tests/ddpg-ros");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        ArchitectureAdapter trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();
        ArchitectureAdapter criticArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/ddpg-ros/comp"), "RosCriticNetwork");

        trainGenerator.generate(modelPath, "RosActorNetwork", trainedArchitecture, criticArchitecture);

        assertEquals(0, Log.getErrorCount());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ros-ddpg"),
                Arrays.asList(
                        "CNNTrainer_rosActorNetwork.py",
                        "CNNLAOptimizer_rosActorNetwork.h",
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

    @Test
    public void defaultHyperparameterValuesAreUsed() {
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
                        "execute_Alexnet",
                        "CNNModelLoader.h"));
    }

    private boolean containsErrorWithMessage(List<Finding> findings, String message) {
        if (findings == null || findings.isEmpty()) return false;
        return findings.stream().anyMatch(finding -> finding.isError() && finding.getMsg().equals(message));
    }
}


