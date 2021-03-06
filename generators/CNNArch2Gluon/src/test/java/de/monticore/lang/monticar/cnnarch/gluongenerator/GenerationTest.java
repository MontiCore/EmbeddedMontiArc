/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionSourceGenerator;
import de.monticore.lang.monticar.cnnarch.gluongenerator.util.NNArchitectureMockFactory;
import de.monticore.lang.monticar.cnntrain._symboltable.NNArchitectureSymbol;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.*;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import org.junit.contrib.java.lang.system.ExpectedSystemExit;

//import static java.sql.DriverManager.println;
import static junit.framework.TestCase.assertTrue;
import static org.mockito.Mockito.mock;
//import static sun.misc.Version.print;

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
                Paths.get("target/generated-sources-cnnarch"),
                Paths.get("src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_CifarClassifierNetwork.py",
                        "CNNNet_CifarClassifierNetwork.py",
                        "CNNDataLoader_CifarClassifierNetwork.py",
                        "CNNSupervisedTrainer_CifarClassifierNetwork.py",
                        "CNNPredictor_CifarClassifierNetwork.h",
                        "execute_CifarClassifierNetwork",
                        "CNNModelLoader.h"));
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
                        "execute_Alexnet",
                        "CNNModelLoader.h"));
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
                        "execute_VGG16",
                        "CNNModelLoader.h"));
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
    public void testRNNtest() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "RNNtest", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testRNNencdec() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "RNNencdec", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testRNNsearch() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "RNNsearch", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testShow_attend_tell() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "Show_attend_tell", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testEpisodicMemoryGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "EpisodicMemoryNetwork", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_EpisodicMemoryNetwork.py",
                        "CNNNet_EpisodicMemoryNetwork.py",
                        "CNNDataLoader_EpisodicMemoryNetwork.py",
                        "CNNSupervisedTrainer_EpisodicMemoryNetwork.py",
                        "CNNPredictor_EpisodicMemoryNetwork.h",
                        "execute_EpisodicMemoryNetwork",
                        "CNNModelLoader.h"));
    }

    @Test
    public void testLoadNetworkLayerGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "LoadNetworkTest", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2GluonCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_LoadNetworkTest.py",
                        "CNNNet_LoadNetworkTest.py",
                        "CNNDataLoader_LoadNetworkTest.py",
                        "CNNSupervisedTrainer_LoadNetworkTest.py",
                        "CNNPredictor_LoadNetworkTest.h",
                        "execute_LoadNetworkTest",
                        "CNNModelLoader.h"));
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
                        "CNNLAOptimizer_fullConfig.h"));
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
                        "CNNLAOptimizer_simpleConfig.h"));
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
                        "CNNLAOptimizer_emptyConfig.h"));
    }

    @Test
    public void testReinforcementConfig2() {
        // given
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        NNArchitectureSymbol trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();

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
        // given
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        NNArchitectureSymbol trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();

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
        NNArchitectureSymbol trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();
        NNArchitectureSymbol criticArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/ddpg/comp"), "CriticNetwork");

        trainGenerator.generate(modelPath, "ActorNetwork", trainedArchitecture, criticArchitecture);

        assertTrue(Log.getFindings().stream().noneMatch(Finding::isError));
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ddpg"),
                Arrays.asList(
                        "CNNTrainer_actorNetwork.py",
                        "CNNLAOptimizer_actorNetwork.h",
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
    public void testTd3Config() {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests/td3");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        NNArchitectureSymbol trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();
        NNArchitectureSymbol criticArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
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
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests/ddpg-ros");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        NNArchitectureSymbol trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();
        NNArchitectureSymbol criticArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/ddpg-ros/comp"), "RosCriticNetwork");

        trainGenerator.generate(modelPath, "RosActorNetwork", trainedArchitecture, criticArchitecture);

        assertTrue(Log.getFindings().stream().noneMatch(Finding::isError));
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
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests/default-gan");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        NNArchitectureSymbol genArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/default-gan/arc"), "DefaultGAN");
        NNArchitectureSymbol disArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
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
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests/info-gan");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);
        NNArchitectureSymbol genArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/info-gan/arc"), "InfoGAN");
        NNArchitectureSymbol disArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
                Paths.get("./src/test/resources/valid_tests/info-gan/arc"), "InfoDiscriminator");
        NNArchitectureSymbol qnetArchitecture = NNArchitectureMockFactory.createArchitectureSymbolByCNNArchModel(
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
}
