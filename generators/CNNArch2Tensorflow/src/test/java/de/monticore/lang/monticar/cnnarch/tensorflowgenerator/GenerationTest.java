/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.tensorflowgenerator;

import de.monticore.lang.monticar.cnnarch.generator.GenerationAbortedException;
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

import static junit.framework.Assert.assertEquals;
import static junit.framework.Assert.fail;
import static junit.framework.TestCase.assertTrue;

public class GenerationTest extends AbstractSymtabTest {

    @Rule
    public final ExpectedSystemExit exit = ExpectedSystemExit.none();

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testCifar10Classifier() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "CifarClassifierNetwork", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2TensorflowCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_CifarClassifierNetwork.py",
                        "CNNDataLoader_CifarClassifierNetwork.py",
                        "CNNPredictor_CifarClassifierNetwork.h",
                        "execute_CifarClassifierNetwork"));
    }

    @Test
    public void testAlexnetGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "Alexnet", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2TensorflowCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_Alexnet.py",
                        "CNNDataLoader_Alexnet.py",
                        "CNNPredictor_Alexnet.h",
                        "execute_Alexnet"));
    }

    @Test
    public void testGeneratorVGG16() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "VGG16", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2TensorflowCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_VGG16.py",
                        "CNNDataLoader_VGG16.py",
                        "CNNPredictor_VGG16.h",
                        "execute_VGG16"));
    }

    @Test
    public void testThreeInputCNNGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ThreeInputCNN_M14"};
        CNNArch2TensorflowCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ResNeXt50"};
        CNNArch2TensorflowCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleStreams() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "MultipleStreams"};
        CNNArch2TensorflowCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testSimpleCfgGeneration() {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Tensorflow trainGenerator = new CNNTrain2Tensorflow();
        trainGenerator.generate(modelPath, "SimpleConfig");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList("CNNTrainer_simpleConfig.py"));
    }

    @Test
    public void testEmptyCfgGeneration() {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Tensorflow trainGenerator = new CNNTrain2Tensorflow();
        trainGenerator.generate(modelPath, "EmptyConfig");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList("CNNTrainer_emptyConfig.py"));
    }

    @Test
    public void testGenerationWithoutTrainingConfigurationFails() {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");

        try {
            CNNTrain2Tensorflow trainGenerator = new CNNTrain2Tensorflow();
            trainGenerator.generate(modelPath, "ModelWithoutTrainingConfiguration");
            fail("A RuntimeException should have been thrown!");
        } catch (RuntimeException e) {
            assertEquals(1, Log.getErrorCount());
            assertEquals("Could not resolve training configuration for model 'ModelWithoutTrainingConfiguration'.", e.getMessage());
        }
    }

    @Test
    public void testSchemaIsCheckedBeforeGenerationStarts() {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");

        try {
            CNNTrain2Tensorflow trainGenerator = new CNNTrain2Tensorflow();
            trainGenerator.generate(modelPath, "InvalidSchemaDefinition");
            fail("A GenerationAbortedException should have been thrown!");
        } catch (GenerationAbortedException e) {
            assertEquals("Generation aborted due to errors in the training configuration.", e.getMessage());
        }
    }

    @Test
    public void testCMakeGeneration() {
        Log.getFindings().clear();
        String rootModelName = "alexnet";
        CNNArch2Tensorflow generator = new CNNArch2Tensorflow();
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
}