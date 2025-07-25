/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch.generator.GenerationAbortedException;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.contrib.java.lang.system.Assertion;
import org.junit.contrib.java.lang.system.ExpectedSystemExit;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;

import static junit.framework.Assert.assertEquals;
import static junit.framework.Assert.fail;
import static junit.framework.TestCase.assertTrue;

public class GenerationTest extends AbstractSymtabTest{
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
        CNNArch2MxNetCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                "CNNCreator_CifarClassifierNetwork.py",
                "CNNPredictor_CifarClassifierNetwork.h",
                "CNNDataCleaner_CifarClassifierNetwork.py",
                "execute_CifarClassifierNetwork",
                "CNNBufferFile.h"));
    }

    @Test
    public void testAlexnetGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "Alexnet", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2MxNetCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_Alexnet.py",
                        "CNNPredictor_Alexnet.h",
                        "CNNDataCleaner_Alexnet.py",
                        "execute_Alexnet"));
    }

    @Test
    public void testGeneratorVGG16() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "VGG16", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2MxNetCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_VGG16.py",
                        "CNNPredictor_VGG16.h",
                        "CNNDataCleaner_VGG16.py",
                        "execute_VGG16"));
    }

    @Test
    public void testThreeInputCNNGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ThreeInputCNN_M14"};
        exit.expectSystemExit();
        exit.checkAssertionAfterwards(new Assertion() {
            public void checkAssertion() {
                assertTrue(Log.getFindings().size() == 2);
            }
        });
        CNNArch2MxNetCli.main(args);
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ResNeXt50"};
        CNNArch2MxNetCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleStreams() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/invalid_tests", "-r", "MultipleStreams"};
        exit.expectSystemExit();
        exit.checkAssertionAfterwards(new Assertion() {
            public void checkAssertion() {
                assertTrue(Log.getFindings().size() == 2);
            }
        });
        CNNArch2MxNetCli.main(args);
    }

    @Test
    public void testMultipleOutputs() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/invalid_tests", "-r", "MultipleOutputs"};
        exit.expectSystemExit();
        exit.checkAssertionAfterwards(new Assertion() {
            public void checkAssertion() {
                assertTrue(Log.getFindings().size() == 2);
            }
        });
        CNNArch2MxNetCli.main(args);
    }

    @Test
    public void testFullCfgGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String sourcePath = "src/test/resources/valid_tests";
        CNNTrain2MxNet trainGenerator = new CNNTrain2MxNet();
        trainGenerator.generate(Paths.get(sourcePath), "FullConfig");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNTrainer_fullConfig.py"));

        // TODO: Fix of test: bring in type info from schema.. at the moment the parameter is parsed as an integer, but in the schema it is a double
        // --> we need schema info in getValue()-Method (or before?)
    }

    @Test
    public void testSimpleCfgGeneration() throws IOException {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2MxNet trainGenerator = new CNNTrain2MxNet();

        trainGenerator.generate(modelPath, "SimpleConfig");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNTrainer_simpleConfig.py"));
    }

    @Test
    public void testEmptyCfgGeneration() throws IOException {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2MxNet trainGenerator = new CNNTrain2MxNet();
        trainGenerator.generate(modelPath, "EmptyConfig");

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNTrainer_emptyConfig.py"));
    }

    @Test
    public void testGenerationWithoutTrainingConfigurationFails() {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");

        try {
            CNNTrain2MxNet trainGenerator = new CNNTrain2MxNet();
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
            CNNTrain2MxNet trainGenerator = new CNNTrain2MxNet();
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
        CNNArch2MxNet generator = new CNNArch2MxNet();
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