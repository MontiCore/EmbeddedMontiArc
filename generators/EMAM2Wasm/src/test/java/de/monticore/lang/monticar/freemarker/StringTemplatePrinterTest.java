/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.freemarker;

import static org.assertj.core.api.Assertions.assertThat;

import freemarker.cache.FileTemplateLoader;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class StringTemplatePrinterTest {

  private static final Path TEMPLATE_DIR = Paths.get("src/test/resources/freemarker");
  private static final String TEMPLATE = "test.ftl";
  private static final Map<String, Object> EMPTY_DATA_MODEL = new HashMap<>();
  private static final String EXPECTED_GENERATED_STRING = "test";

  @Nested
  class Process {

    Template template;

    @BeforeEach
    void setUp() throws IOException {
      FileTemplateLoader ftl = new FileTemplateLoader(TEMPLATE_DIR.toFile());
      TemplateFactory templateFactory = new TemplateFactory(ftl);
      template = templateFactory.getTemplate(TEMPLATE);
    }

    @Test
    void shouldGenerateToFile() throws IOException, TemplateException {
      StringTemplatePrinter templatePrinter = new StringTemplatePrinter(template);

      templatePrinter.process(EMPTY_DATA_MODEL);

      assertThat(templatePrinter.toString()).isEqualTo(EXPECTED_GENERATED_STRING);
    }
  }

  @Nested
  class Print {

    Template template;

    @BeforeEach
    void setUp() throws IOException {
      FileTemplateLoader ftl = new FileTemplateLoader(TEMPLATE_DIR.toFile());
      TemplateFactory templateFactory = new TemplateFactory(ftl);
      template = templateFactory.getTemplate(TEMPLATE);
    }

    @Test
    void shouldGenerateToFile() throws IOException, TemplateException {
      StringTemplatePrinter templatePrinter = new StringTemplatePrinter(template);

      assertThat(templatePrinter.print(EMPTY_DATA_MODEL)).isEqualTo(EXPECTED_GENERATED_STRING);
    }
  }
}
