/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.web;

import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.doThrow;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import de.monticore.lang.monticar.generator.html.HtmlGenerator;
import de.monticore.lang.monticar.generator.js.JsGenerator;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class WebStepTest {

  private static final Path SOME_DIRECTORY = Paths.get("some/directory");
  private static final Path SOME_WASM_FILE = Paths.get("some/wasm/file.js");
  private static final WrapperNameProvider WRAPPER_NAME_PROVIDER = new JsWrapperNameProvider();
  private static final InterfaceNameProvider INTERFACE_NAME_PROVIDER = new HtmlInterfaceNameProvider();
  private static final String SOME_MODEL_NAME = "someModel";
  private static final ExpandedComponentInstanceSymbol NULL_MODEL = null;
  private static final Path NULL_PATH = null;
  private static final Path EXPECTED_WRAPPER_FILE_PATH = Paths
      .get("some/directory/someModel_wrapper.js");
  private static final Path EXPECTED_HTML_FILE_PATH = Paths.get("some/directory/someModel.html");
  private static final JsGeneratorFactory NULL_JS_GENERATOR_FACTORY = null;
  private static final HtmlGeneratorFactory NULL_HTML_GENERATOR_FACTORY = null;
  private static final WrapperNameProvider NULL_WRAPPER_NAME_PROVIDER = null;
  private static final InterfaceNameProvider NULL_INTERFACE_NAME_PROVIDER = null;

  private JsGenerator jsGenerator;
  private HtmlGenerator htmlGenerator;
  private JsGeneratorFactory jsGeneratorFactory;
  private HtmlGeneratorFactory htmlGeneratorFactory;
  private ExpandedComponentInstanceSymbol model;

  @BeforeEach
  void setUp() {
    jsGenerator = mock(JsGenerator.class);
    jsGeneratorFactory = mock(JsGeneratorFactory.class);
    when(jsGeneratorFactory.getJsGenerator(any())).thenReturn(jsGenerator);

    htmlGenerator = mock(HtmlGenerator.class);
    htmlGeneratorFactory = mock(HtmlGeneratorFactory.class);
    when(htmlGeneratorFactory.getHtmlGenerator(any(), any(), any())).thenReturn(htmlGenerator);

    model = mock(ExpandedComponentInstanceSymbol.class);
    when(model.getName()).thenReturn(SOME_MODEL_NAME);
  }

  @Nested
  class Constructor {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenJsGeneratorFactoryIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> new WebStep(NULL_JS_GENERATOR_FACTORY, htmlGeneratorFactory,
                SOME_DIRECTORY, WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER));
      }

      @Test
      void whenHtmlGeneratorFactoryIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> new WebStep(jsGeneratorFactory, NULL_HTML_GENERATOR_FACTORY,
                SOME_DIRECTORY, WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER));
      }

      @Test
      void whenWebDirIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> new WebStep(jsGeneratorFactory, htmlGeneratorFactory,
                NULL_PATH, WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER));
      }

      @Test
      void whenWrapperNameProviderIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> new WebStep(jsGeneratorFactory, htmlGeneratorFactory,
                SOME_DIRECTORY, NULL_WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER));
      }

      @Test
      void whenInterfaceNameProviderIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> new WebStep(jsGeneratorFactory, htmlGeneratorFactory,
                SOME_DIRECTORY, WRAPPER_NAME_PROVIDER, NULL_INTERFACE_NAME_PROVIDER));
      }
    }
  }

  @Nested
  class GenerateWebInterface {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenModelIsNull() {
        WebStep webStep = new WebStep(jsGeneratorFactory, htmlGeneratorFactory, SOME_DIRECTORY,
            WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER);

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> webStep.generateWebInterface(NULL_MODEL, SOME_WASM_FILE));
      }

      @Test
      void whenWasmFileIsNull() {
        WebStep webStep = new WebStep(jsGeneratorFactory, htmlGeneratorFactory, SOME_DIRECTORY,
            WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER);

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> webStep.generateWebInterface(model, NULL_PATH));
      }
    }

    @Nested
    class ShouldThrowWebInterfaceGenerationException {

      @Test
      void whenJsGeneratorThrowsIOException() throws IOException, TemplateException {
        doThrow(new IOException()).when(jsGenerator).generate(any());
        WebStep webStep = new WebStep(jsGeneratorFactory, htmlGeneratorFactory, SOME_DIRECTORY,
            WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER);

        assertThatExceptionOfType(WebInterfaceGeneratorException.class)
            .isThrownBy(() -> webStep.generateWebInterface(model, SOME_WASM_FILE));
      }

      @Test
      void whenJsGeneratorThrowsTemplateException() throws IOException, TemplateException {
        doThrow(new TemplateException(null)).when(jsGenerator).generate(any());
        WebStep webStep = new WebStep(jsGeneratorFactory, htmlGeneratorFactory, SOME_DIRECTORY,
            WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER);

        assertThatExceptionOfType(WebInterfaceGeneratorException.class)
            .isThrownBy(() -> webStep.generateWebInterface(model, SOME_WASM_FILE));
      }

      @Test
      void whenHtmlGeneratorThrowsIOException() throws IOException, TemplateException {
        doThrow(new IOException()).when(htmlGenerator).generate(any());
        WebStep webStep = new WebStep(jsGeneratorFactory, htmlGeneratorFactory, SOME_DIRECTORY,
            WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER);

        assertThatExceptionOfType(WebInterfaceGeneratorException.class)
            .isThrownBy(() -> webStep.generateWebInterface(model, SOME_WASM_FILE));
      }

      @Test
      void whenHtmlGeneratorThrowsTemplateException() throws IOException, TemplateException {
        doThrow(new TemplateException(null)).when(htmlGenerator).generate(any());
        WebStep webStep = new WebStep(jsGeneratorFactory, htmlGeneratorFactory, SOME_DIRECTORY,
            WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER);

        assertThatExceptionOfType(WebInterfaceGeneratorException.class)
            .isThrownBy(() -> webStep.generateWebInterface(model, SOME_WASM_FILE));
      }
    }

    @Nested
    class WhenParametersNotNull {

      @Test
      void shouldCallJsGeneratorFactoryWithWrapperFilePath() {
        WebStep webStep = new WebStep(jsGeneratorFactory, htmlGeneratorFactory, SOME_DIRECTORY,
            WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER);

        webStep.generateWebInterface(model, SOME_WASM_FILE);

        verify(jsGeneratorFactory).getJsGenerator(EXPECTED_WRAPPER_FILE_PATH);
      }

      @Test
      void shouldCallHtmlFactoryWithCorrectArguments() {
        WebStep webStep = new WebStep(jsGeneratorFactory, htmlGeneratorFactory, SOME_DIRECTORY,
            WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER);

        webStep.generateWebInterface(model, SOME_WASM_FILE);

        verify(htmlGeneratorFactory).getHtmlGenerator(eq(EXPECTED_HTML_FILE_PATH), any(), any());
      }

      @Test
      void shouldGenerateJs() throws IOException, TemplateException {
        WebStep webStep = new WebStep(jsGeneratorFactory, htmlGeneratorFactory, SOME_DIRECTORY,
            WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER);

        webStep.generateWebInterface(model, SOME_WASM_FILE);

        verify(jsGenerator).generate(model);
      }

      @Test
      void shouldGenerateHtml() throws IOException, TemplateException {
        WebStep webStep = new WebStep(jsGeneratorFactory, htmlGeneratorFactory, SOME_DIRECTORY,
            WRAPPER_NAME_PROVIDER, INTERFACE_NAME_PROVIDER);

        webStep.generateWebInterface(model, SOME_WASM_FILE);

        verify(htmlGenerator).generate(model);
      }
    }
  }

}
