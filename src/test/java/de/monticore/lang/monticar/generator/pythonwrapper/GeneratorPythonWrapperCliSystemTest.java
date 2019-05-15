package de.monticore.lang.monticar.generator.pythonwrapper;

import com.google.common.collect.Lists;
import de.se_rwth.commons.logging.Log;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;
import de.monticore.lang.monticar.generator.pythonwrapper.util.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.pythonwrapper.util.FileUtils;

import java.net.URI;
import java.net.URISyntaxException;
import java.nio.file.Paths;
import java.util.List;

import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.assertj.core.api.Fail.fail;

/**
 *
 */
public class GeneratorPythonWrapperCliSystemTest {
    private GeneratorPythonWrapperCli cli;

    private static String PATH_TO_CALCULATOR_MODEL;

    static {
        try {
            URI uriToCalculatorModel =
                    GeneratorPythonWrapperCliSystemTest.class.getClassLoader().getResource("calculator").toURI();
            PATH_TO_CALCULATOR_MODEL = Paths.get(uriToCalculatorModel).toString();
        } catch (URISyntaxException e) {
            fail(e.getMessage());
        }
    }

    private static final String OUTPUT_DIRECTORY = Paths.get("generated-test-files").toString();

    @Before
    public void setup() {
        Log.enableFailQuick(false);
        cli = new GeneratorPythonWrapperCli();
    }

    @AfterClass
    public static void tearDown() {
        FileUtils.deleteDirectory(OUTPUT_DIRECTORY);
    }

    @Test
    public void testCalculatorModel() {
        // given
        String[] args = {"-m", PATH_TO_CALCULATOR_MODEL, "-r", "calculator.calculator", "-o", OUTPUT_DIRECTORY};

        // when
        cli.run(args);

        // then
        List<String> expectedFiles = Lists.newArrayList(
                "calculator_calculator_executor.h",
                "calculator_calculator_executor.cpp",
                "calculator_calculator_executor.i",
                "CMakeLists.txt"
        );

        AbstractSymtabTest.checkFilesAreEqual(
                Paths.get(PATH_TO_CALCULATOR_MODEL,"calculator", "pythonwrapper"),
                Paths.get(OUTPUT_DIRECTORY),
                expectedFiles);
    }

    @Test
    public void testCalculatorModelWithNoOutputArgument() {
        // given
        String[] args = {"-m", PATH_TO_CALCULATOR_MODEL, "-r", "calculator.calculator"};

        // when
        cli.run(args);

        // then
        List<String> expectedFiles = Lists.newArrayList(
                "calculator_calculator_executor.h",
                "calculator_calculator_executor.cpp",
                "calculator_calculator_executor.i",
                "CMakeLists.txt"
        );

        AbstractSymtabTest.checkFilesAreEqual(
                Paths.get(PATH_TO_CALCULATOR_MODEL, "calculator", "pythonwrapper"),
                Paths.get(PATH_TO_CALCULATOR_MODEL),
                expectedFiles);
    }

    @Test
    public void whenNoModelPathIsGivenThenExceptionIsThrown() {
        // given
        String[] args = {"-r", "calculator.calculator", "-o", OUTPUT_DIRECTORY};

        // then
        assertThatExceptionOfType(IllegalArgumentException.class)
                .isThrownBy(() -> cli.run(args))
                .withMessage("Missing required option: m");
    }

    @Test
    public void whenNoRootModelIsGivenThenExceptionIsThrown() {
        // given
        String[] args = {"-m", PATH_TO_CALCULATOR_MODEL, "-o", OUTPUT_DIRECTORY};

        // then
        assertThatExceptionOfType(IllegalArgumentException.class)
                .isThrownBy(() -> cli.run(args))
                .withMessage("Missing required option: r");
    }

    @Test
    public void whenNoArgumentsAreGivenThenExceptionIsThrown() {
        // given
        String[] args = {};

        // then
        assertThatExceptionOfType(IllegalArgumentException.class)
                .isThrownBy(() -> cli.run(args))
                .withMessage("Missing required options: m, r");
    }

    @Test
    public void whenOnlyOutputDirectoryIsGivenThenExceptionIsThrown() {
        // given
        String[] args = {"-o", OUTPUT_DIRECTORY};

        // then
        assertThatExceptionOfType(IllegalArgumentException.class)
                .isThrownBy(() -> cli.run(args))
                .withMessage("Missing required options: m, r");
    }
}