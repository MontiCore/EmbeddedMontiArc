/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.generator.pythonwrapper.template.configuration.PythonWrapperTemplateConfigurationProvider;
import de.monticore.lang.monticar.generator.pythonwrapper.template.configuration.TemplateConfigurationProvider;
import de.monticore.lang.monticar.generator.pythonwrapper.template.model.CppVariableViewModel;
import de.monticore.lang.monticar.generator.pythonwrapper.template.model.CppWrapperViewModel;
import freemarker.template.Configuration;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import java.net.URL;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.fail;

/**
 *
 */
public class TemplateLoaderTest {
    private final static String TEST_FILE_HEADER = "test_case_executor_header.h";
    private final static String TEST_FILE_HEADER_NO_INPUT = "test_case_executor_header_no_input.h";
    private final static String TEST_FILE_HEADER_NO_OUTPUT = "test_case_executor_header_no_output.h";
    private final static String TEST_FILE_HEADER_NO_INPUT_OUTPUT = "test_case_executor_header_no_input_output.h";
    private final static String TEST_FILE_CPP = "test_case_executor_cpp.cpp";
    private final static String TEST_FILE_CPP_NO_INPUT = "test_case_executor_cpp_no_input.cpp";
    private final static String TEST_FILE_CPP_NO_OUTPUT = "test_case_executor_cpp_no_output.cpp";
    private final static String TEST_FILE_CPP_NO_INPUT_OUTPUT = "test_case_executor_cpp_no_input_output.cpp";
    private final static String TEST_FILE_SWIG_INTERFACE =  "test_case_executor_interface.i";
    private final static String TEST_FILE_CMAKE = "test_case_cmake.txt";

    private final String ACTUAL_EXECUTOR_HEADER = loadTestFile(TEST_FILE_HEADER);
    private final String ACTUAL_EXECUTOR_HEADER_NO_INPUT = loadTestFile(TEST_FILE_HEADER_NO_INPUT);
    private final String ACTUAL_EXECUTOR_HEADER_NO_OUTPUT = loadTestFile(TEST_FILE_HEADER_NO_OUTPUT);
    private final String ACTUAL_EXECUTOR_HEADER_NO_INPUT_OUTPUT = loadTestFile(TEST_FILE_HEADER_NO_INPUT_OUTPUT);
    private final String ACTUAL_EXECUTOR_CPP = loadTestFile(TEST_FILE_CPP);
    private final String ACTUAL_EXECUTOR_CPP_NO_INPUT = loadTestFile(TEST_FILE_CPP_NO_INPUT);
    private final String ACTUAL_EXECUTOR_CPP_NO_OUTPUT = loadTestFile(TEST_FILE_CPP_NO_OUTPUT);
    private final String ACTUAL_EXECUTOR_CPP_NO_INPUT_OUTPUT = loadTestFile(TEST_FILE_CPP_NO_INPUT_OUTPUT);
    private final String ACTUAL_SWIG_INTERFACE = loadTestFile(TEST_FILE_SWIG_INTERFACE);
    private final String ACTUAL_CMAKE = loadTestFile(TEST_FILE_CMAKE);

    private static final List<CppVariableViewModel> INPUTS = Lists.newArrayList(
            new CppVariableViewModel("int", "anyName1"),
            new CppVariableViewModel("arma::colvec", "anyName2"),
            new CppVariableViewModel("arma::cube", "anyName3")
    );

    private static final List<CppVariableViewModel> OUTPUTS = Lists.newArrayList(
            new CppVariableViewModel("arma::imat", "anyName4"),
            new CppVariableViewModel("arma::vec", "anyName5")
    );
    private static final String WRAPPER_CLASS_NAME = "any_component_executor";
    private static final String WRAPPED_COMPONENT_NAME = "any_component";
    private static final String INPUT_CLASS_NAME = "any_component_input";
    private static final String OUTPUT_CLASS_NAME = "any_component_output";

    private TemplateLoader templateLoader;

    private final TemplateConfigurationProvider configurationProvider = new PythonWrapperTemplateConfigurationProvider();
    private final Configuration configuration = configurationProvider.getConfiguration();

    private CppWrapperViewModel viewData;
    private Writer writer;

    @Before
    public void setup() {
        templateLoader = new TemplateLoader(configuration);
        viewData = new CppWrapperViewModel(
                WRAPPER_CLASS_NAME,
                WRAPPED_COMPONENT_NAME,
                INPUT_CLASS_NAME,
                OUTPUT_CLASS_NAME,
                INPUTS,
                OUTPUTS
        );
        writer = new StringWriter();
    }

    @Test
    public void whenProcessWrapperHeaderThenReturnedStringShouldBeEqualToTestFile() {
        // when
        try {
            this.templateLoader.processWrapperHeaderTemplate(this.viewData, this.writer);
        } catch (IOException | TemplateException e) {
            fail(e.getMessage());
        }
        String result = writer.toString();

        // then
        assertThat(result).isEqualTo(ACTUAL_EXECUTOR_HEADER);
    }

    @Test
    public void whenProcessWrapperHeaderWithNoInputThenReturnedStringShouldBeEqualToTestFile() {
        // given
        CppWrapperViewModel viewDataWithoutInput = new CppWrapperViewModel(
                WRAPPER_CLASS_NAME,
                WRAPPED_COMPONENT_NAME,
                INPUT_CLASS_NAME,
                OUTPUT_CLASS_NAME,
                new ArrayList<>(),
                OUTPUTS
        );

        // when
        try {
            this.templateLoader.processWrapperHeaderTemplate(viewDataWithoutInput, this.writer);
        } catch (IOException | TemplateException e) {
            fail(e.getMessage());
        }
        String result = writer.toString();

        // then
        assertThat(result).isEqualTo(ACTUAL_EXECUTOR_HEADER_NO_INPUT);
    }

    @Test
    public void whenProcessWrapperHeaderWithNoOutputThenReturnedStringShouldBeEqualToTestFile() {
        // given
        CppWrapperViewModel viewDataWithoutOutput = new CppWrapperViewModel(
                WRAPPER_CLASS_NAME,
                WRAPPED_COMPONENT_NAME,
                INPUT_CLASS_NAME,
                OUTPUT_CLASS_NAME,
                INPUTS,
                new ArrayList<>()
        );

        // when
        try {
            this.templateLoader.processWrapperHeaderTemplate(viewDataWithoutOutput, this.writer);
        } catch (IOException | TemplateException e) {
            fail(e.getMessage());
        }
        String result = writer.toString();

        // then
        assertThat(result).isEqualTo(ACTUAL_EXECUTOR_HEADER_NO_OUTPUT);
    }

    @Test
    public void whenProcessWrapperHeaderWithNoInputAndOutputThenReturnedStringShouldBeEqualToTestFile() {
        // given
        CppWrapperViewModel viewDataWithoutInputAndOutput = new CppWrapperViewModel(
                WRAPPER_CLASS_NAME,
                WRAPPED_COMPONENT_NAME,
                INPUT_CLASS_NAME,
                OUTPUT_CLASS_NAME,
                new ArrayList<>(),
                new ArrayList<>()
        );

        // when
        try {
            this.templateLoader.processWrapperHeaderTemplate(viewDataWithoutInputAndOutput, this.writer);
        } catch (IOException | TemplateException e) {
            fail(e.getMessage());
        }
        String result = writer.toString();

        // then
        assertThat(result).isEqualTo(ACTUAL_EXECUTOR_HEADER_NO_INPUT_OUTPUT);
    }

    @Test
    public void whenProcessWrapperCppThenReturnedStringShouldBeEqualToTestFile() {
        // when
        try {
            this.templateLoader.processWrapperCppTemplate(this.viewData, this.writer);
        } catch (IOException | TemplateException e) {
            fail(e.getMessage());
        }
        String result = writer.toString();

        // then
        assertThat(result).isEqualTo(ACTUAL_EXECUTOR_CPP);
    }

    @Test
    public void whenProcessWrapperCppWithNoInputThenReturnedStringShouldBeEqualToTestFile() {
        // given
        CppWrapperViewModel viewDataWithoutInput = new CppWrapperViewModel(
                WRAPPER_CLASS_NAME,
                WRAPPED_COMPONENT_NAME,
                INPUT_CLASS_NAME,
                OUTPUT_CLASS_NAME,
                new ArrayList<>(),
                OUTPUTS
        );

        // when
        try {
            this.templateLoader.processWrapperCppTemplate(viewDataWithoutInput, this.writer);
        } catch (IOException | TemplateException e) {
            fail(e.getMessage());
        }
        String result = writer.toString();

        // then
        assertThat(result).isEqualTo(ACTUAL_EXECUTOR_CPP_NO_INPUT);
    }

    @Test
    public void whenProcessWrapperCppWithNoOutputThenReturnedStringShouldBeEqualToTestFile() {
        // given
        CppWrapperViewModel viewDataWithoutOutput = new CppWrapperViewModel(
                WRAPPER_CLASS_NAME,
                WRAPPED_COMPONENT_NAME,
                INPUT_CLASS_NAME,
                OUTPUT_CLASS_NAME,
                INPUTS,
                new ArrayList<>()
        );

        // when
        try {
            this.templateLoader.processWrapperCppTemplate(viewDataWithoutOutput, this.writer);
        } catch (IOException | TemplateException e) {
            fail(e.getMessage());
        }
        String result = writer.toString();

        // then
        assertThat(result).isEqualTo(ACTUAL_EXECUTOR_CPP_NO_OUTPUT);
    }

    @Test
    public void whenProcessWrapperCppWithNoInputAndOutputThenReturnedStringShouldBeEqualToTestFile() {
        // given
        CppWrapperViewModel viewDataWithoutInputAndOutput = new CppWrapperViewModel(
                WRAPPER_CLASS_NAME,
                WRAPPED_COMPONENT_NAME,
                INPUT_CLASS_NAME,
                OUTPUT_CLASS_NAME,
                new ArrayList<>(),
                new ArrayList<>()
        );

        // when
        try {
            this.templateLoader.processWrapperCppTemplate(viewDataWithoutInputAndOutput, this.writer);
        } catch (IOException | TemplateException e) {
            fail(e.getMessage());
        }
        String result = writer.toString();

        // then
        assertThat(result).isEqualTo(ACTUAL_EXECUTOR_CPP_NO_INPUT_OUTPUT);
    }

    @Test
    public void whenProcessSwigInterfaceThenReturnedStringShouldBeEqualToTestFile() {
        // when
        try {
            this.templateLoader.processSwigTemplate(this.viewData, this.writer);
        } catch (IOException | TemplateException e) {
            fail(e.getMessage());
        }
        String result = writer.toString();

        // then
        assertThat(result).isEqualTo(ACTUAL_SWIG_INTERFACE);
    }

    @Test
    public void whenProcessCmakeThenReturnedStringShouldBeEqualToTestFile() {
        // when
        try {
            this.templateLoader.processCmakeTemplate(this.viewData, this.writer);
        } catch (IOException | TemplateException e) {
            fail(e.getMessage());
        }
        String result = writer.toString();

        // then
        assertThat(result).isEqualTo(ACTUAL_CMAKE);
    }

    private String loadTestFile(final String fileName) {
        String fileContent = "";

        URL urlToFile = this.getClass()
                .getResource("/wrapper-generation-test-files/" + fileName);
        String pathToFile = urlToFile.getPath();
        File file = new File(pathToFile);

        try {
            fileContent = new String(Files.readAllBytes(file.toPath()));
        } catch (IOException e) {
            e.printStackTrace();
            fail("Cannot read " + fileName);
        }

        return fileContent;
    }
}
