/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper;

import com.google.common.collect.Lists;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;
import de.monticore.lang.monticar.generator.pythonwrapper.util.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.pythonwrapper.util.ComponentInstanceSymbolLibrary;
import de.monticore.lang.monticar.generator.pythonwrapper.util.ComponentInstanceSymbolLibraryFactory;
import de.monticore.lang.monticar.generator.pythonwrapper.util.FileUtils;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.file.Paths;
import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.fail;
import static org.assertj.core.api.Assumptions.assumeThat;

/**
 *
 */
public class GeneratorPythonWrapperIntegrationTest {
    private static final String TEST_OUTPUT_DIRECTORY = "generated-test-files";
    private static final String ARMANPY_DIRECTORY_STRING = "armanpy";

    private final static List<String> ARMANPY_PATH_FILES = Lists.newArrayList(
            Paths.get(ARMANPY_DIRECTORY_STRING, "armanpy.hpp").toString(),
            Paths.get(ARMANPY_DIRECTORY_STRING, "armanpy.i").toString(),
            Paths.get(ARMANPY_DIRECTORY_STRING,"armanpy_1d.i").toString(),
            Paths.get(ARMANPY_DIRECTORY_STRING, "armanpy_2d.i").toString(),
            Paths.get(ARMANPY_DIRECTORY_STRING, "armanpy_3d.i").toString(),
            Paths.get(ARMANPY_DIRECTORY_STRING, "numpy.i").toString()
    );

    private GeneratorPythonWrapper generatorPythonWrapper;
    private final ComponentInstanceSymbolLibrary componentLibrary
            = new ComponentInstanceSymbolLibraryFactory().getComponentInstanceLibrary();

    @Before
    public void setup() {
        File outputDir = new File(TEST_OUTPUT_DIRECTORY);
        if (!outputDir.exists()) {
            assumeThat(outputDir.mkdirs()).isTrue();
        }

        this.generatorPythonWrapper = new GeneratorPythonWrapperFactory().create();
    }

    @AfterClass
    public static void tearDown() {
        FileUtils.deleteDirectory(TEST_OUTPUT_DIRECTORY);
    }

    @Test
    public void calculatorModelTest() {
        // given
        final String outputDirectory = Paths.get(TEST_OUTPUT_DIRECTORY, "calculator").toString();
        final String pathToTestCaseFiles = getPathStringFromJavaResourcePath("calculator/calculator/pythonwrapper");
        this.generatorPythonWrapper.setGenerationTargetPath(outputDirectory);
        EMAComponentInstanceSymbol componentInstanceSymbol = componentLibrary.getModelByIdentifier(
                ComponentInstanceSymbolLibraryFactory.CALCULATOR_MODEL);

        // when
        List<File> resultFiles = null;
        try {
            resultFiles = this.generatorPythonWrapper.generateFiles(componentInstanceSymbol);
        } catch (IOException e) {
            fail(e.getMessage());
        }

        // then
        List<String> expectedFiles = Lists.newArrayList(
                "calculator_calculator_executor.h",
                "calculator_calculator_executor.cpp",
                "calculator_calculator_executor.i",
                "CMakeLists.txt"
        );
        expectedFiles.addAll(ARMANPY_PATH_FILES);

        assertThat(resultFiles).allMatch(File::exists);
        AbstractSymtabTest.checkFilesAreEqual(
                Paths.get(pathToTestCaseFiles),
                Paths.get(outputDirectory),
                expectedFiles);
    }

    @Test
    public void allTypesModelTest() {
        // given
        final String outputDirectory = Paths.get(TEST_OUTPUT_DIRECTORY, "all-types-model").toString();
        final String pathToTestCaseFiles = getPathStringFromJavaResourcePath("all-types-model/types/pythonwrapper");
        this.generatorPythonWrapper.setGenerationTargetPath(outputDirectory);
        EMAComponentInstanceSymbol componentInstanceSymbol = componentLibrary.getModelByIdentifier(
                ComponentInstanceSymbolLibraryFactory.ALL_TYPES_MODEL);

        // when
        List<File> resultFiles = null;
        try {
            resultFiles = this.generatorPythonWrapper.generateFiles(componentInstanceSymbol);
        } catch (IOException e) {
            fail(e.getMessage());
        }

        // then
        List<String> expectedFiles = Lists.newArrayList(
                "types_allTypes_executor.h",
                "types_allTypes_executor.cpp",
                "types_allTypes_executor.i",
                "CMakeLists.txt"
        );
        expectedFiles.addAll(ARMANPY_PATH_FILES);

        assertThat(resultFiles).allMatch(File::exists);
        AbstractSymtabTest.checkFilesAreEqual(
                Paths.get(pathToTestCaseFiles),
                Paths.get(outputDirectory),
                expectedFiles);
    }

    @Test
    public void noInputModelTest() {
        // given
        final String outputDirectory = Paths.get(TEST_OUTPUT_DIRECTORY, "noinput").toString();
        final String pathToTestCaseFiles = getPathStringFromJavaResourcePath("noinput/test/pythonwrapper");
        this.generatorPythonWrapper.setGenerationTargetPath(outputDirectory);
        EMAComponentInstanceSymbol componentInstanceSymbol = componentLibrary.getModelByIdentifier(
                ComponentInstanceSymbolLibraryFactory.NO_INPUT_MODEL);

        // when
        List<File> resultFiles = null;
        try {
            resultFiles = this.generatorPythonWrapper.generateFiles(componentInstanceSymbol);
        } catch (IOException e) {
            fail(e.getMessage());
        }

        // then
        List<String> expectedFiles = Lists.newArrayList(
                "test_noInputComponent_executor.h",
                "test_noInputComponent_executor.cpp",
                "test_noInputComponent_executor.i",
                "CMakeLists.txt"
        );
        expectedFiles.addAll(ARMANPY_PATH_FILES);

        assertThat(resultFiles).allMatch(File::exists);
        AbstractSymtabTest.checkFilesAreEqual(
                Paths.get(pathToTestCaseFiles),
                Paths.get(outputDirectory),
                expectedFiles);
    }

    @Test
    public void noOutputModelTest() {
        // given
        final String outputDirectory = Paths.get(TEST_OUTPUT_DIRECTORY,"nooutput").toString();
        String pathToTestCaseFiles = getPathStringFromJavaResourcePath("nooutput/test/pythonwrapper");
        this.generatorPythonWrapper.setGenerationTargetPath(outputDirectory);
        EMAComponentInstanceSymbol componentInstanceSymbol = componentLibrary.getModelByIdentifier(
                ComponentInstanceSymbolLibraryFactory.NO_OUTPUT_MODEL);

        // when
        List<File> resultFiles = null;
        try {
            resultFiles = this.generatorPythonWrapper.generateFiles(componentInstanceSymbol);
        } catch (IOException e) {
            fail(e.getMessage());
        }

        // then
        List<String> expectedFiles = Lists.newArrayList(
                "test_noOutputComponent_executor.h",
                "test_noOutputComponent_executor.cpp",
                "test_noOutputComponent_executor.i",
                "CMakeLists.txt"
        );
        expectedFiles.addAll(ARMANPY_PATH_FILES);

        assertThat(resultFiles).allMatch(File::exists);
        AbstractSymtabTest.checkFilesAreEqual(
                Paths.get(pathToTestCaseFiles),
                Paths.get(outputDirectory),
                expectedFiles);
    }

    @Test
    public void noInputNoOutputModelTest() {
        // given
        final String outputDirectory = Paths.get(TEST_OUTPUT_DIRECTORY, "noinput-nooutput").toString();
        final String pathToTestCaseFiles = getPathStringFromJavaResourcePath("noinputnooutput/test/pythonwrapper");
        this.generatorPythonWrapper.setGenerationTargetPath(outputDirectory);
        EMAComponentInstanceSymbol componentInstanceSymbol = componentLibrary.getModelByIdentifier(
                ComponentInstanceSymbolLibraryFactory.NO_INPUT_NO_OUTPUT_MODEL);

        // when
        List<File> resultFiles = null;
        try {
            resultFiles = this.generatorPythonWrapper.generateFiles(componentInstanceSymbol);
        } catch (IOException e) {
            fail(e.getMessage());
        }

        // then
        List<String> expectedFiles = Lists.newArrayList(
                "test_noInputNoOutputComponent_executor.h",
                "test_noInputNoOutputComponent_executor.cpp",
                "test_noInputNoOutputComponent_executor.i",
                "CMakeLists.txt"
        );
        expectedFiles.addAll(ARMANPY_PATH_FILES);

        assertThat(resultFiles).allMatch(File::exists);
        AbstractSymtabTest.checkFilesAreEqual(
                Paths.get(pathToTestCaseFiles),
                Paths.get(outputDirectory),
                expectedFiles);
    }

    @Test
    public void deepLearningComponentModelTest() {
        // given
        final String outputDirectory = Paths.get(TEST_OUTPUT_DIRECTORY,"/deep-learning-component").toString();
        final String pathToTestCaseFiles = getPathStringFromJavaResourcePath("dl-model/dl/pythonwrapper");
        this.generatorPythonWrapper.setGenerationTargetPath(outputDirectory);
        EMAComponentInstanceSymbol componentInstanceSymbol = componentLibrary.getModelByIdentifier(
                ComponentInstanceSymbolLibraryFactory.DEEP_LEARNING_COMPONENT);

        // when
        List<File> resultFiles = null;
        try {
            resultFiles = this.generatorPythonWrapper.generateFiles(componentInstanceSymbol);
        } catch (IOException e) {
            fail(e.getMessage());
        }

        // then
        List<String> expectedFiles = Lists.newArrayList(
                "dl_deepLearningComponent_executor.h",
                "dl_deepLearningComponent_executor.cpp",
                "dl_deepLearningComponent_executor.i",
                "CMakeLists.txt"
        );
        expectedFiles.addAll(ARMANPY_PATH_FILES);

        assertThat(resultFiles).allMatch(File::exists);
        AbstractSymtabTest.checkFilesAreEqual(
                Paths.get(pathToTestCaseFiles),
                Paths.get(outputDirectory),
                expectedFiles);
    }

    @Test
    public void willAcceptCalculator() {
        // given
        EMAComponentInstanceSymbol componentInstanceSymbol = componentLibrary.getModelByIdentifier(
                ComponentInstanceSymbolLibraryFactory.CALCULATOR_MODEL);

        // when
        boolean result = this.generatorPythonWrapper.willAccept(componentInstanceSymbol);

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void willAcceptAllTypes() {
        // given
        EMAComponentInstanceSymbol componentInstanceSymbol = componentLibrary.getModelByIdentifier(
                ComponentInstanceSymbolLibraryFactory.ALL_TYPES_MODEL);

        // when
        boolean result = this.generatorPythonWrapper.willAccept(componentInstanceSymbol);

        // then
        assertThat(result).isTrue();
    }

    private String getPathStringFromJavaResourcePath(final String javaResourcePath) {
        URI uriToResoureceFile;
        try {
            uriToResoureceFile = this.getClass().getClassLoader().getResource(javaResourcePath).toURI();
        } catch (URISyntaxException e) {
            fail("Cannot load classpath");
            return null;
        }
        return Paths.get(uriToResoureceFile).toString();
    }
}
