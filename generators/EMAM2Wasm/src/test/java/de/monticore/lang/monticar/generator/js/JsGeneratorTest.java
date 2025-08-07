/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.js;

import static org.mockito.ArgumentMatchers.any;
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
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class JsGeneratorTest {

  private static final Path RESOLVING_BASE_DIR = Paths.get("src/test/resources/jsfilegeneration");
  private static final String RANGE_MODEL = "models.multiplePorts";

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
    void shouldGenerateCpp() throws IOException, TemplateException {
      TemplateProcessor templateProcessor = mock(TemplateProcessor.class);
      JsGenerator jsGenerator = new JsGenerator(templateProcessor);

      jsGenerator.generate(rangeModel);

      verify(templateProcessor).process(any());
    }
  }
}
