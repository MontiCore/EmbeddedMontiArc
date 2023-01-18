/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.testing.AutomaticStreamTestGenerator;
import de.monticore.lang.monticar.generator.testing.OSHelper;
import de.monticore.lang.monticar.generator.testing.StreamTestExecution;
import de.monticore.lang.monticar.generator.testing.StreamTestModifier;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.SystemUtils;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;

import static org.junit.Assert.assertNotNull;

/**
 * TODO README
 * To understand how automatic test generation works, examine the tests in this class.
 *
 */
@Ignore
public class AutomaticStreamTestGenerationTest extends AbstractSymtabTest {
    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
        String execName = "";
        if (SystemUtils.IS_OS_WINDOWS) {
            try {
                execName = "substN.bat";
                Process p = Runtime.
                        getRuntime().
                        exec(execName);
                while (p.isAlive()) {
                    if (Log.isInfoEnabled("")) {
                        System.out.print((char) p.getInputStream().read());
                    }
                }

            } catch (Exception ex) {
                ex.printStackTrace();
            }
        }
    }

    @Test
    public void testStreamTestAutopilotTestGen() throws Exception {
        AutomaticStreamTestGenerator generator = new AutomaticStreamTestGenerator();
        generator.generateTests("de.rwth.armin.modeling.autopilot.autopilot",
                "src/test/resources", OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/autopilot/", "1", 10);

    }

    @Test
    public void testStreamTestAutopilotAllComponentsTestGen() throws Exception {
        AutomaticStreamTestGenerator generator = new AutomaticStreamTestGenerator();
        generator.generateTests("",
                "src/test/resources/emastudio/autopilot", OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/autopilot/", "1", 10);
    }

    @Test
    public void testStreamTestPacmanControllerSimpleTestGenAndExec() throws Exception {
        AutomaticStreamTestGenerator generator = new AutomaticStreamTestGenerator();
        generator.generateTests("de.rwth.pacman.pacManControllerSimple",
                "src/test/resources/emastudio/pacman", OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/pacman/", "1", 10);
        //testFilesAreEqual(files, restPath); generated values are random*/
        testGenCPPFilesAndExec("./target/generated-sources-cpp/streamtest", "/pacman",
                "./src/test/resources/emastudio/pacman", "./target/generated-sources-cpp/streamtest/pacman",
                "de.rwth.pacman.pacManControllerSimple",
                "de.rwth.pacman.PacManControllerSimpleTest1");
    }

    @Ignore//Does not work in maven for some reason
    @Test
    public void testStreamTestObjectDetectorTestGenAndExec() throws Exception {
        AutomaticStreamTestGenerator automaticGenerator = new AutomaticStreamTestGenerator();

        automaticGenerator.generateTests("detection.objectDetector1",
                "src/test/resources/emastudio/cluster", OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/cluster/", "1", 1);

        testGenCPPFilesAndExec("./target/generated-sources-cpp/streamtest", "/cluster",
                "./src/test/resources/emastudio/cluster", "./target/generated-sources-cpp/streamtest/cluster",
                "detection.objectDetector1",
                "detection.ObjectDetector1Test1");
    }

    //Create image test manually, as generation for these large matrices takes a lot of time
    @Ignore
    @Test
    public void testStreamTestClustererAllComponentsTestGen() throws Exception {
        AutomaticStreamTestGenerator generator = new AutomaticStreamTestGenerator();
        generator.generateTests("de.rwth.clustering.detection.objectDetector1",
                "src/test/resources/emastudio/clustering", OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/clustering/",
                "1", 1);
       /* TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/emastudio/clustering");

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useStreamTestTestGeneration("1", 1);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamtest/cluster/");
        generatorCPP.useArmadilloBackend();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources"));
        List<File> files = generatorCPP.generateFiles(symtab, null, symtab);
        String restPath = "streamtest/cluster";
        TaggingResolver streamSymtab = createSymTabAndTaggingResolver("./target/generated-sources-cpp/streamtest/cluster");
        generatorCPP.setGenerateTests(true);
        generatorCPP.setModelsDirPath(Paths.get(getDirPrefix()+"/target/generated-sources-cpp/streamtest/cluster"));
        generatorCPP.saveFilesToDisk(generatorCPP.handleTestAndCheckDir(streamSymtab));*/
        //testFilesAreEqual(files, restPath); generated values are random
        testGenCPPFilesAndExec("./target/generated-sources-cpp/streamtest", "/cluster",
                "./src/test/resources/emastudio/cluster", "./target/generated-sources-cpp/streamtest/cluster",
                "de.rwth.armin.modeling.autopilot.motion.calculatePidError",
                "de.rwth.armin.modeling.autopilot.motion.CalculatePidErrorTest1");

    }

    //Create image test manually, as generation for these large matrices takes a lot of time
    @Ignore
    @Test
    public void testStreamTestAutopilotSteam2CPPTestGen() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/emastudio/autopilot");

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useStreamTestTestGeneration("1", 1);
        generatorCPP.setGenerationTargetPath(OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/autopilot/");
        generatorCPP.useArmadilloBackend();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources/emastudio/autopilot"));
        /*List<File> files = generatorCPP.generateFiles(symtab, null, symtab);
        String restPath = "streamtest/cluster";*/
        //TaggingResolver streamSymtab = createSymTabAndTaggingResolver("./target/generated-sources-cpp/streamtest/cluster");
        generatorCPP.setGenerateTests(true);
        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("de.rwth.armin.modeling.autopilot.motion.calculatePidError", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        generatorCPP.generateFiles(symtab, componentSymbol);
        //generatorCPP.setModelsDirPath(Paths.get("./target/generated-sources-cpp/streamtest/cluster"));
        // generatorCPP.saveFilesToDisk(generatorCPP.handleTestAndCheckDir(symtab));
        //testFilesAreEqual(files, restPath); generated values are random
    }

    /***
     * If no error is logged, it was used correctly(usually log.error terminates vm in the se logger)
     */
    @Test
    public void testCLISimpleExample() throws Exception {
        // --models-dir="%HOME%\model\autopilot" ^
        //   --output-dir="%TESTS_CPP_DIR%" ^
        //   --root-model=%1 ^
        //   --flag-generate-tests ^
        //   --flag-use-armadillo-backend
//        System.out.println("Generation Done");
        String targetBasePath = OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest";
        String targetRestPath = "/autopilot";
        String targetFullPath = targetBasePath + targetRestPath;
        String modelDirectory = OSHelper.getDirPrefix() + "/src/test/resources/emastudio/autopilot";
        String outputDirectory = OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/autopilot";
        String fullComponentInstanceName = "de.rwth.armin.modeling.autopilot.motion.calculatePidError";
        String fullStreamTestName = "de.rwth.armin.modeling.autopilot.motion.CalculatePidErrorTest1";
        String fullStreamTestPathName = fullStreamTestName.replaceAll("\\.", "\\/");
        String args[] = {"--models-dir=" + modelDirectory,
                "--output-dir=" + outputDirectory,
                "--root-model=" + fullComponentInstanceName,
                "--flag-generate-tests",
                "--flag-use-armadillo-backend"};
        GeneratorCppCli.main(args);
    }

    @Test
    public void testCLIExample() throws Exception {
        AutomaticStreamTestGenerator generator = new AutomaticStreamTestGenerator();
        generator.generateTests("",
                "src/test/resources/emastudio/autopilot", OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/autopilot/", "1", 10);

        /*String targetBasePath = "./target/generated-sources-cpp/streamtest";
        String targetRestPath = "/autopilot";
        String targetFullPath = targetBasePath + targetRestPath;
        String modelDirectory = "./src/test/resources/emastudio/autopilot";
        String outputDirectory = "./target/generated-sources-cpp/streamtest/autopilot";
        String fullComponentInstanceName = "de.rwth.armin.modeling.autopilot.motion.calculatePidError";
        String fullStreamTestName = "de.rwth.armin.modeling.autopilot.motion.CalculatePidErrorTest1";
        String fullStreamTestPathName = fullStreamTestName.replaceAll("\\.", "\\/");
        String args[] = {"--models-dir=" + outputDirectory,
                "--output-dir=" + outputDirectory,
                "--root-model=" + fullComponentInstanceName,
                "--flag-generate-tests",
                "--flag-use-armadillo-backend"};
        File srcDir = new File(modelDirectory);
        File destDir = new File(outputDirectory);
        FileUtils.copyDirectory(srcDir, destDir);
        GeneratorCppCli.main(args);
        StreamTestExecution.compileTests(targetFullPath, targetBasePath);
        StreamTestExecution.executeTests(targetBasePath);

        //Execute again to check if tests pass

        StreamTestModifier.updateStreamTestWithResults(getDirPrefix()+"/target/generated-sources-cpp/streamtest/autopilot/" + fullStreamTestPathName + ".stream"
                , getDirPrefix()+"/target/generated-sources-cpp/streamtest/exec/" + fullStreamTestName);
        GeneratorCppCli.main(args);

        StreamTestExecution.compileTests(targetFullPath, targetBasePath);
        StreamTestExecution.executeTests(targetBasePath);*/
        testGenCPPFilesAndExec("./target/generated-sources-cpp/streamtest", "/autopilot",
                "./src/test/resources/emastudio/autopilot", "./target/generated-sources-cpp/streamtest/autopilot",
                "de.rwth.armin.modeling.autopilot.motion.calculatePidError",
                "de.rwth.armin.modeling.autopilot.motion.CalculatePidErrorTest1");
        testGenCPPFilesAndExec("./target/generated-sources-cpp/streamtest", "/autopilot",
                "./src/test/resources/emastudio/autopilot", "./target/generated-sources-cpp/streamtest/autopilot",
                "de.rwth.armin.modeling.autopilot.common.compass2CurrentDirection",
                "de.rwth.armin.modeling.autopilot.common.Compass2CurrentDirectionTest1");
    }

    @Test
    public void testLoggingExample1() throws Exception {
        AutomaticStreamTestGenerator generator = new AutomaticStreamTestGenerator();
        generator.generateTests("",
                "src/test/resources/emastudio/autopilot", OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/autopilot/", "1", 10);

        testGenCPPFilesAndExecWithExecLogging("./target/generated-sources-cpp/streamtest", "/autopilot",
                "./src/test/resources/emastudio/autopilot", "./target/generated-sources-cpp/streamtest/autopilot",
                "de.rwth.armin.modeling.autopilot.motion.calculatePidError",
                "de.rwth.armin.modeling.autopilot.motion.CalculatePidErrorTest1");
    }

    @Test
    public void testLoggingExample2() throws Exception {
        AutomaticStreamTestGenerator generator = new AutomaticStreamTestGenerator();
        generator.generateTests("",
                "src/test/resources/emastudio/autopilot", OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/autopilot/", "1", 10);

        testGenCPPFilesAndExecWithExecLogging("./target/generated-sources-cpp/streamtest", "/autopilot",
                "./src/test/resources/emastudio/autopilot", "./target/generated-sources-cpp/streamtest/autopilot",
                "de.rwth.armin.modeling.autopilot.motion.calculateEngineAndBrakes",
                "de.rwth.armin.modeling.autopilot.motion.CalculateEngineAndBrakesTest1");
    }

    @Ignore //Does not work in maven for some reason
    @Test
    public void testComponent() throws Exception {
        setUp();
        AutomaticStreamTestGenerator automaticGenerator = new AutomaticStreamTestGenerator();

        automaticGenerator.generateTests("detection.objectDetector1",
                "src/test/resources/emastudio/cluster", OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/cluster/", "1", 1);

        testGenCPPFilesAndExec("./target/generated-sources-cpp/streamtest", "/cluster",
                "./src/test/resources/emastudio/cluster", "./target/generated-sources-cpp/streamtest/cluster",
                "detection.objectDetector1",
                "detection.ObjectDetector1Test1");
    }

    @Test
    public void testMultipleAssignments() throws Exception {
        String fullComponentInstanceName = "de.multipleAssignments";
        String modelDir = "src/test/resources/streamtests/multipleAssignmentsBase";
        String targetBasePath = "./target/generated-sources-cpp/streamtest";
        String targetRestPath = "/multipleAssignments";
        String streamTestBaseName = "de.MultipleAssignmentsTest";

        executeTest(fullComponentInstanceName, streamTestBaseName, modelDir, targetBasePath, targetRestPath, "1", 1, true);
    }

    public void testGenCPPFilesAndExec(String targetBasePath, String targetRestPath, String modelDirectory, String outputDirectory,
                                       String fullComponentInstanceName, String fullStreamTestName) throws Exception {
        String targetFullPath = targetBasePath + targetRestPath;
        String fullStreamTestPathName = fullStreamTestName.replaceAll("\\.", "\\/");
        String args[] = {"--models-dir=" + outputDirectory,
                "--output-dir=" + outputDirectory,
                "--root-model=" + fullComponentInstanceName,
                "--flag-generate-tests",
                "--flag-use-armadillo-backend"};
        File srcDir = new File(modelDirectory);
        File destDir = new File(outputDirectory);
        FileUtils.copyDirectory(srcDir, destDir);
        GeneratorCppCli.main(args);
        if( !SystemUtils.IS_OS_WINDOWS && !SystemUtils.IS_OS_LINUX){
            Log.warn("Unsupported OS for StreamTestExecution");
            return;
        }
        StreamTestExecution.compileTests(targetFullPath, targetBasePath);
        StreamTestExecution.executeTests(targetBasePath);

        //Execute again to check if tests pass

        StreamTestModifier.updateStreamTestWithResults(OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/" + targetRestPath + "/" + fullStreamTestPathName + ".stream"
                , "./target/generated-sources-cpp/streamtest/exec/" + fullStreamTestName);
        GeneratorCppCli.main(args);

        StreamTestExecution.compileTests(targetFullPath, targetBasePath);
        StreamTestExecution.executeTests(targetBasePath);

    }

    public void testGenCPPFilesAndExecWithExecLogging(String targetBasePath, String targetRestPath, String modelDirectory, String outputDirectory,
                                                      String fullComponentInstanceName, String fullStreamTestName) throws Exception {
        String targetFullPath = targetBasePath + targetRestPath;
        String fullStreamTestPathName = fullStreamTestName.replaceAll("\\.", "\\/");
        String args[] = {"--models-dir=" + outputDirectory,
                "--output-dir=" + outputDirectory,
                "--root-model=" + fullComponentInstanceName,
                "--flag-generate-tests",
                "--flag-use-armadillo-backend",
                "--flag-use-exec-logging"};
        File srcDir = new File(modelDirectory);
        File destDir = new File(outputDirectory);
        FileUtils.copyDirectory(srcDir, destDir);
        GeneratorCppCli.main(args);
        if( !SystemUtils.IS_OS_WINDOWS && !SystemUtils.IS_OS_LINUX){
            Log.warn("Unsupported OS for StreamTestExecution");
            return;
        }
        StreamTestExecution.compileTests(targetFullPath, targetBasePath);
        StreamTestExecution.executeTests(targetBasePath);

        //Execute again to check if tests pass

        StreamTestModifier.updateStreamTestWithResults(OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/" + targetRestPath + "/" + fullStreamTestPathName + ".stream"
                , "./target/generated-sources-cpp/streamtest/exec/" + fullStreamTestName);
        GeneratorCppCli.main(args);

        StreamTestExecution.compileTests(targetFullPath, targetBasePath);
        StreamTestExecution.executeTests(targetBasePath);

    }
    //Maybe add test that executes all stream tests in the resource dir(will take longer than an hour to execute) later

    private void executeTest(String fullComponentInstanceName, String streamTestBaseName, String modelDir, String targetBasePath, String targetRestPath, String testNamePostfix, int amountTickValues, boolean logging) throws Exception {
        AutomaticStreamTestGenerator generator = new AutomaticStreamTestGenerator();
        String targetPath = targetBasePath + targetRestPath;
        generator.generateTests(fullComponentInstanceName,
                modelDir, targetPath, testNamePostfix, amountTickValues);

        if (logging) {
            testGenCPPFilesAndExecWithExecLogging(targetBasePath, targetRestPath,
                    modelDir, targetPath,
                    fullComponentInstanceName,
                    streamTestBaseName + testNamePostfix);
        } else {
            testGenCPPFilesAndExec(targetBasePath, targetRestPath,
                    modelDir, targetPath,
                    fullComponentInstanceName,
                    streamTestBaseName + testNamePostfix);
        }
    }
}
