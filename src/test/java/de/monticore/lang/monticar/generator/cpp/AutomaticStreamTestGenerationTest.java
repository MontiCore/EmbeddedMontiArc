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
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.testing.AutomaticStreamTestGenerator;
import de.monticore.lang.monticar.generator.testing.StreamTestExecution;
import de.monticore.lang.monticar.generator.testing.StreamTestModifier;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.Options;
import org.apache.commons.io.FileUtils;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * TODO README
 * To understand how automatic test generation works, examine the tests in this class.
 *
 * @author Sascha Schneiders
 */
public class AutomaticStreamTestGenerationTest extends AbstractSymtabTest {
    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
        
    }

    @Test
    public void testStreamTestAutopilotTestGen() throws Exception {
        AutomaticStreamTestGenerator generator = new AutomaticStreamTestGenerator();
        generator.generateTests("de.rwth.armin.modeling.autopilot.autopilot",
                "src/test/resources", "./target/generated-sources-cpp/streamtest/autopilot/", "1", 10);

    }

    @Test
    public void testStreamTestAutopilotAllComponentsTestGen() throws Exception {
        AutomaticStreamTestGenerator generator = new AutomaticStreamTestGenerator();
        generator.generateTests("",
                "src/test/resources/emastudio/autopilot", "./target/generated-sources-cpp/streamtest/autopilot/", "1", 10);
    }

    @Test
    public void testStreamTestPacmanControllerSimpleTestGen() throws Exception {
        AutomaticStreamTestGenerator generator = new AutomaticStreamTestGenerator();
        generator.generateTests("de.rwth.pacman.pacManControllerSimple",
                "src/test/resources/emastudio/pacman", "./target/generated-sources-cpp/streamtest/pacman/", "1", 10);
/*
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/emastudio/pacman");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useStreamTestTestGeneration("1", 10);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamtest/pacman/");
        generatorCPP.useArmadilloBackend();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources"));
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "streamtest/pacman";
        //testFilesAreEqual(files, restPath); generated values are random*/
    }

    //Create image test manually, as generation for these large matrices takes a lot of time
    @Ignore
    @Test
    public void testStreamTestClustererAllComponentsTestGen() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/emastudio/clustering");

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useStreamTestTestGeneration("1", 1);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamtest/cluster/");
        generatorCPP.useArmadilloBackend();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources"));
        /*List<File> files = generatorCPP.generateFiles(symtab, null, symtab);
        String restPath = "streamtest/cluster";*/
        TaggingResolver streamSymtab = createSymTabAndTaggingResolver("./target/generated-sources-cpp/streamtest/cluster");
        generatorCPP.setGenerateTests(true);
        generatorCPP.setModelsDirPath(Paths.get("./target/generated-sources-cpp/streamtest/cluster"));
        generatorCPP.saveFilesToDisk(generatorCPP.handleTestAndCheckDir(streamSymtab));
        //testFilesAreEqual(files, restPath); generated values are random
    }

    //Create image test manually, as generation for these large matrices takes a lot of time
    @Ignore
    @Test
    public void testStreamTestAutopilotSteam2CPPTestGen() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/emastudio/autopilot");

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useStreamTestTestGeneration("1", 1);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamtest/autopilot/");
        generatorCPP.useArmadilloBackend();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources/emastudio/autopilot"));
        /*List<File> files = generatorCPP.generateFiles(symtab, null, symtab);
        String restPath = "streamtest/cluster";*/
        //TaggingResolver streamSymtab = createSymTabAndTaggingResolver("./target/generated-sources-cpp/streamtest/cluster");
        generatorCPP.setGenerateTests(true);
        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("de.rwth.armin.modeling.autopilot.motion.calculatePidError", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        generatorCPP.generateFiles(symtab, componentSymbol, symtab);
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
        System.out.println("Generation Done");
        String targetBasePath = "/target/generated-sources-cpp/streamtest";
        String targetRestPath = "/autopilot";
        String targetFullPath = targetBasePath + targetRestPath;
        String modelDirectory = "./src/test/resources/emastudio/autopilot";
        String outputDirectory = "./target/generated-sources-cpp/streamtest/autopilot";
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
        testStreamTestAutopilotAllComponentsTestGen();
        String targetBasePath = "/target/generated-sources-cpp/streamtest";
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

        StreamTestModifier.updateStreamTestWithResults("./target/generated-sources-cpp/streamtest/autopilot/" + fullStreamTestPathName + ".stream"
                , "./target/generated-sources-cpp/streamtest/exec/" + fullStreamTestName);
        GeneratorCppCli.main(args);

        StreamTestExecution.compileTests(targetFullPath, targetBasePath);
        StreamTestExecution.executeTests(targetBasePath);
    }

//TODO add test that executes all stream tests in the resource dir(will take long to execute)
}
