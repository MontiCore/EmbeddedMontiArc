/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.building;

import de.monticore.lang.monticar.generator.pythonwrapper.util.FileUtils;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.net.URISyntaxException;
import java.nio.file.Path;
import java.nio.file.Paths;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.fail;
import static org.junit.Assume.assumeTrue;


public class PythonModuleBuilderTest {
    private static final String ROOT_OUTPUT_DIRECTORY_NAME = "generated-test-files";
    private static final Path OUTPUT_DIRECTORY = Paths.get(ROOT_OUTPUT_DIRECTORY_NAME, "modlib");

    private PythonModuleBuilder pythonModuleBuilder;

    @Before
    public void setup() {
        pythonModuleBuilder = new PythonModuleBuilder();
        assumeTrue(pythonModuleBuilder.checkPythonModuleBuildAvailable());
    }

    @Test
    public void shouldBuildPythonModule() {
        // given
        Path sourceDirectory = null;
        String componentName = "torcs_agent_dqn_reward";
        try {
            sourceDirectory = Paths.get(PythonModuleBuilderTest.class.getClassLoader()
                    .getResource("generated-wrapper/pylib").toURI());
        } catch (URISyntaxException e) {
            fail("Cannot load test files");
        }

        // when
        try {
            pythonModuleBuilder.buildPythonModule(sourceDirectory, componentName, OUTPUT_DIRECTORY);
        } catch (PythonModuleBuildingException e) {
            fail("Test failed: " + e.getMessage());
        }

        // then
        File expectedModuleFile = Paths.get(OUTPUT_DIRECTORY.toString(), componentName + "_executor.py").toFile();
        File expectedLibraryFile = Paths.get(OUTPUT_DIRECTORY.toString(), "_" + componentName + "_executor.so").toFile();
        assertThat(expectedLibraryFile).exists();
        assertThat(expectedModuleFile).exists();
    }

    @After
    public void cleanup() {
        File directory = new File(ROOT_OUTPUT_DIRECTORY_NAME);
        if (directory.exists()) {
            FileUtils.deleteDirectory(directory);
        }
    }
}
