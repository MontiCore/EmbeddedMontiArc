/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.html;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.freemarker.TemplateProcessor;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.ResolverFactory;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import java.util.function.Function;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

class HtmlGeneratorTest {

  private static final Path RESOLVING_BASE_DIR = Paths.get("src/test/resources/jsfilegeneration");
  private static final String RANGE_MODEL = "models.multiplePorts";
  private static final String SOME_WASM_NAME = "model.wasm";
  private static final String SOME_WRAPPER_NAME = "wrapper.js";

  private static final Function<ExpandedComponentInstanceSymbol, String> WASM_NAMING_FUNCITON
      = model -> SOME_WASM_NAME;
  private static final Function<ExpandedComponentInstanceSymbol, String> WRAPPER_NAMING_FUNCITON
      = model -> SOME_WRAPPER_NAME;

  @Nested
  class Generate {

    ExpandedComponentInstanceSymbol rangeModel;

    @BeforeEach
    void setUp() {
      ResolverFactory resolverFactory = new ResolverFactory(RESOLVING_BASE_DIR);
      Resolver resolver = resolverFactory.get();
      rangeModel = resolver.getExpandedComponentInstanceSymbol(RANGE_MODEL);
    }

    @Test
    @SuppressWarnings("unchecked")
    void shouldGenerateHtml() throws IOException, TemplateException {
      TemplateProcessor templateProcessor = mock(TemplateProcessor.class);
      HtmlGenerator htmlGenerator = new HtmlGenerator(templateProcessor, WASM_NAMING_FUNCITON,
          WRAPPER_NAMING_FUNCITON);

      htmlGenerator.generate(rangeModel);

      ArgumentCaptor<Map<String, ?>> argument = ArgumentCaptor.forClass(Map.class);
      verify(templateProcessor).process(argument.capture());
      Map<String, ?> map = argument.getValue();
      assertThat(map).containsKeys("model_name", "model", "model_wrapper", "inports", "outports");
    }
  }

}
