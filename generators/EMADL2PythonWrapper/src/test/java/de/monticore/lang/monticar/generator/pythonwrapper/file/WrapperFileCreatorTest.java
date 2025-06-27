/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.file;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import de.monticore.lang.monticar.generator.pythonwrapper.util.FileUtils;

import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.fail;
import static de.monticore.lang.monticar.generator.pythonwrapper.util.FileUtils.loadFileFromJavaContext;

/**
 *
 */
public class WrapperFileCreatorTest {
    private final static String TEST_OUTPUT_DIRECTORY = Paths.get("generated-test-files").toString();
    private final static String ARMANPY_RESOURCE_DIRECTORY
            = "de/monticore/lang/monticar/generator/pythonwrapper/file/armanpy";
    private final static Integer NUMBER_OF_ARMANPY_FILES = 6;

    private final static String TEST_FILE_A = "testFileA.h";
    private final static String TEST_FILE_B = "testFileB.cpp";
    private final static String TEST_FILE_C = "testFileC.i";

    private final static String TEST_FILE_A_CONTENT = "This is test file A";
    private final static String TEST_FILE_B_CONTENT = "This is test file B";
    private final static String TEST_FILE_C_CONTENT = "This is test file C";

    private final static File ARMANPY_HPP = loadFileFromJavaContext(ARMANPY_RESOURCE_DIRECTORY + "/" + "armanpy.hpp");
    private final static File ARMANPY_I = loadFileFromJavaContext(ARMANPY_RESOURCE_DIRECTORY + "/" + "armanpy.i");
    private final static File ARMANPY_1D_I = loadFileFromJavaContext(ARMANPY_RESOURCE_DIRECTORY + "/" + "armanpy_1d.i");
    private final static File ARMANPY_2D_I = loadFileFromJavaContext(ARMANPY_RESOURCE_DIRECTORY + "/" + "armanpy_2d.i");
    private final static File ARMANPY_3D_I = loadFileFromJavaContext(ARMANPY_RESOURCE_DIRECTORY + "/" + "armanpy_3d.i");
    private final static File NUMPY_I = loadFileFromJavaContext(ARMANPY_RESOURCE_DIRECTORY + "/" + "numpy.i");

    private WrapperFileCreator wrapperFileCreator;

    @Before
    public void setup() {
        wrapperFileCreator = new WrapperFileCreator();
        wrapperFileCreator.setOutputDirectory(TEST_OUTPUT_DIRECTORY);
    }

    @After
    public void cleanup() {
        File directory = new File(TEST_OUTPUT_DIRECTORY);
        if (directory.exists()) {
            FileUtils.deleteDirectory(directory);
        }
    }

    @Test
    public void whenCreateWrapperFilesWithContentMapThenExpectedFilesAreCreated() {
        // given
        Map<String, String> fileContentMap = new HashMap<>();
        fileContentMap.put(TEST_FILE_A, TEST_FILE_A_CONTENT);
        fileContentMap.put(TEST_FILE_B, TEST_FILE_B_CONTENT);
        fileContentMap.put(TEST_FILE_C, TEST_FILE_C_CONTENT);

        // when
        List<File> resultFiles = null;
        try {
            resultFiles = this.wrapperFileCreator.createWrapperFiles(fileContentMap);
        } catch (IOException e) {
            fail(e.getMessage());
        }

        // then
        assertThat(resultFiles).hasSize(NUMBER_OF_ARMANPY_FILES + fileContentMap.size());
        assertThat(resultFiles).filteredOn(File::exists).hasSize(NUMBER_OF_ARMANPY_FILES + fileContentMap.size());
        assertThat(resultFiles.stream().map(File::getPath)).contains(
                Paths.get(TEST_OUTPUT_DIRECTORY, TEST_FILE_A).toString(),
                Paths.get(TEST_OUTPUT_DIRECTORY, TEST_FILE_B).toString(),
                Paths.get(TEST_OUTPUT_DIRECTORY, TEST_FILE_C).toString(),
                Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "armanpy.hpp").toString(),
                Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "armanpy.i").toString(),
                Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "armanpy_1d.i").toString(),
                Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "armanpy_2d.i").toString(),
                Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "armanpy_3d.i").toString(),
                Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "numpy.i").toString()
        );

        assertThat(resultFiles.stream()
                .filter(f -> f.getPath().equals(Paths.get(TEST_OUTPUT_DIRECTORY, TEST_FILE_A).toString())).findAny().get())
                .hasContent(TEST_FILE_A_CONTENT);

        assertThat(resultFiles.stream()
                .filter(f -> f.getPath().equals(Paths.get(TEST_OUTPUT_DIRECTORY, TEST_FILE_B).toString())).findAny().get())
                .hasContent(TEST_FILE_B_CONTENT);

        assertThat(resultFiles.stream()
                .filter(f -> f.getPath().equals(Paths.get(TEST_OUTPUT_DIRECTORY, TEST_FILE_C).toString())).findAny().get())
                .hasContent(TEST_FILE_C_CONTENT);

        assertThat(resultFiles.stream()
                .filter(f -> f.getPath().equals(Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "armanpy.hpp").toString())).findAny().get())
                .hasSameContentAs(ARMANPY_HPP);

        assertThat(resultFiles.stream()
                .filter(f -> f.getPath().equals(Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "armanpy.i").toString())).findAny().get())
                .hasSameContentAs(ARMANPY_I);

        assertThat(resultFiles.stream()
                .filter(f -> f.getPath().equals(Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "armanpy_1d.i").toString())).findAny().get())
                .hasSameContentAs(ARMANPY_1D_I);

        assertThat(resultFiles.stream()
                .filter(f -> f.getPath().equals(Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "armanpy_2d.i").toString())).findAny().get())
                .hasSameContentAs(ARMANPY_2D_I);

        assertThat(resultFiles.stream()
                .filter(f -> f.getPath().equals(Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "armanpy_3d.i").toString())).findAny().get())
                .hasSameContentAs(ARMANPY_3D_I);

        assertThat(resultFiles.stream()
                .filter(f -> f.getPath().equals(Paths.get(TEST_OUTPUT_DIRECTORY, "armanpy", "numpy.i").toString())).findAny().get())
                .hasSameContentAs(NUMPY_I);
    }
}
