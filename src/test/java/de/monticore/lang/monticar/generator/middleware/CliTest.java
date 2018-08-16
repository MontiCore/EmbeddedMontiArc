package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.se_rwth.commons.logging.Finding;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.nio.file.Files;
import java.nio.file.Paths;

import static org.junit.Assert.assertTrue;


public class CliTest{
    private static final String VALID_MODELS_DIR_OPTION = "--models-dir=src/test/resources/";
    private static final String VALID_ROOT_MODEL_OPTION = "--root-model=tests.a.addComp";
    private static final String VALID_GENERATOR_CPP_OPTION = "--generators=cpp";
    private static final String VALID_GENERATOR_ALL_OPTION = "--generators=cpp,roscpp,odv";

    private static final String INVALID_MODELS_DIR_OPTION = "--models-dir=src/invalid/resources/";
    private static final String INVALID_ROOT_MODEL_OPTION = "--root-model=invalid.invalid.addComp";
    private static final String INVALID_GENERATOR_OPTION = "--generators=invalid";
    private static final String INVALID_GENERATOR_EMPTY_OPTION = "--generators=";

    @BeforeClass
    public static void initLog(){
        LogConfig.init();
        LogConfig.enableFailQuick(false);
    }

    @Before
    public void clearFindings(){
        LogConfig.getFindings().clear();
    }

    @AfterClass
    public static void resetLog(){
        LogConfig.getFindings().clear();
        LogConfig.enableFailQuick(true);
    }


    @Test
    public void testParserError(){
        String[] args = {
                "invalidParsingArg"};

        DistributedTargetGeneratorCli.main(args);
        assertTrue(LogConfig.getErrorCount() == 1);
        assertTrue(LogConfig.getFindings().stream().map(Finding::getMsg).anyMatch(msg -> msg.contains("0x9A1AC")));
    }


    @Test
    public void testNoGenerators(){
        String[] args = {
                VALID_MODELS_DIR_OPTION,
                VALID_ROOT_MODEL_OPTION,
                INVALID_GENERATOR_EMPTY_OPTION,
                "--output-dir=target/cliTest/NoGenerators/"};
        DistributedTargetGeneratorCli.main(args);
        assertTrue(LogConfig.getErrorCount() == 1);
        assertTrue(LogConfig.getFindings().stream().map(Finding::getMsg).anyMatch(msg -> msg.contains("0xE28B6")));
    }

    @Test
    public void testInvalidGenerator(){
        String[] args = {
                VALID_MODELS_DIR_OPTION,
                VALID_ROOT_MODEL_OPTION,
                INVALID_GENERATOR_OPTION,
                "--output-dir=target/cliTest/InvalidGenerators/"};
        DistributedTargetGeneratorCli.main(args);
        assertTrue(LogConfig.getErrorCount() == 1);
        assertTrue(logContains("0xE28B6"));
    }

    @Test
    public void testOneGenerator(){
        String targetDir = "target/cliTest/OneGenerator/";
        String[] args = {
                VALID_MODELS_DIR_OPTION,
                VALID_ROOT_MODEL_OPTION,
                VALID_GENERATOR_CPP_OPTION,
                "--output-dir=" + targetDir};
        DistributedTargetGeneratorCli.main(args);


        String[] positiveFileNames = {
                "CMakeLists.txt",
                "tests_a_addComp/cpp/tests_a_addComp.h",
                "tests_a_addComp/cpp/CMakeLists.txt",
                "tests_a_addComp/coordinator/CMakeLists.txt",
                "tests_a_addComp/coordinator/Coordinator_tests_a_addComp.cpp"
        };

        for (String positiveFileName : positiveFileNames) {
            assertTrue(Files.exists(Paths.get(targetDir + positiveFileName)));
        }
    }

    @Test
    public void testAllGenerators(){
        String targetDir = "target/cliTest/AllGenerators/";
        String[] args = {
                VALID_MODELS_DIR_OPTION,
                VALID_ROOT_MODEL_OPTION,
                VALID_GENERATOR_ALL_OPTION,
                "--output-dir=" + targetDir};
        DistributedTargetGeneratorCli.main(args);


        String[] positiveFileNames = {
                "CMakeLists.txt",
                "tests_a_addComp/cpp/tests_a_addComp.h",
                "tests_a_addComp/cpp/CMakeLists.txt",
                "tests_a_addComp/coordinator/CMakeLists.txt",
                "tests_a_addComp/coordinator/Coordinator_tests_a_addComp.cpp",
                "tests_a_addComp/roscpp/RosAdapter_tests_a_addComp.h",
                "tests_a_addComp/roscpp/CMakeLists.txt",
        };

        for (String positiveFileName : positiveFileNames) {
            assertTrue(Files.exists(Paths.get(targetDir + positiveFileName)));
        }
    }

    @Test
    public void testInvalidRootModelError(){
        String[] args = {
                VALID_MODELS_DIR_OPTION,
                INVALID_ROOT_MODEL_OPTION,
                VALID_GENERATOR_CPP_OPTION,
                "--output-dir=target/cliTest/InvalidRootModel/"};
        DistributedTargetGeneratorCli.main(args);
        assertTrue(LogConfig.getErrorCount() == 1);
        assertTrue(logContains("0x5FFAE"));
    }

    @Test
    public void testInvalidModelsDirError(){
        String[] args = {
                INVALID_MODELS_DIR_OPTION,
                VALID_ROOT_MODEL_OPTION,
                VALID_GENERATOR_CPP_OPTION,
                "--output-dir=target/cliTest/InvalidModelsDir/"};
        DistributedTargetGeneratorCli.main(args);
        assertTrue(LogConfig.getErrorCount() == 1);
        assertTrue(logContains("0x6444B"));
    }


    private boolean logContains(String errorCode){
        return LogConfig.getFindings().stream().map(Finding::getMsg).anyMatch(msg -> msg.contains(errorCode));
    }
}
