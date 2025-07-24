/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.quicktheories.QuickTheory.qt;
import static org.quicktheories.generators.SourceDSL.strings;

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

class CppGeneratorTest {

  private static final Path RESOLVING_BASE_DIR = Paths.get("src/test/resources/cppfilegeneration");
  private static final String ANY_MODEL = "models.scalarPort";

  private static final String ANY_FULL_NAME = "some.full.name";
  private static final String EXPECTED_CPP_CLASS_NAME = "some_full_name";

  @Nested
  class Generate {

    ExpandedComponentInstanceSymbol model;

    @BeforeEach
    void setUp() {
      ResolverFactory resolverFactory = new ResolverFactory(RESOLVING_BASE_DIR);
      Resolver resolver = resolverFactory.get();
      model = resolver.getExpandedComponentInstanceSymbol(ANY_MODEL);
    }

    @Test
    void shouldGenerateCpp() throws IOException, TemplateException {
      TemplateProcessor templateProcessor = mock(TemplateProcessor.class);
      CppGenerator cppGenerator = new CppGenerator(templateProcessor);

      cppGenerator.generate(model);

      verify(templateProcessor).process(any());
    }
  }

  @Nested
  class GetCppClassName {

    @Nested
    class WithNameWithoutDots {

      @Test
      void shouldReturnSameName() {
        qt().forAll(strings().basicLatinAlphabet().ofLengthBetween(1, 10))
            .assuming(name -> !name.contains("."))
            .check(name -> name.equals(CppGenerator.getCppClassName(name)));
      }
    }

    @Nested
    class WithComponentFullName {

      @Test
      void shouldReplaceDots() {
        assertThat(CppGenerator.getCppClassName(ANY_FULL_NAME)).isEqualTo(EXPECTED_CPP_CLASS_NAME);
      }
    }
  }



}
