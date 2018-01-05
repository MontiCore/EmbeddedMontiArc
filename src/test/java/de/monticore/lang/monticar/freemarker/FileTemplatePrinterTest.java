package de.monticore.lang.monticar.freemarker;

import static org.assertj.core.api.Assertions.assertThat;

import de.monticore.lang.monticar.util.FileUtils;
import de.monticore.lang.monticar.util.TextFile;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class FileTemplatePrinterTest {

  private static final Path TEMPLATE_DIR = Paths.get("src/test/resources/freemarker");
  private static final String TEMPLATE = "test.ftl";
  private static final Path OUT_FILE = Paths.get("target/freemarker/test.txt");
  private static final Map<String, Object> EMPTY_DATA_MODEL = new HashMap<>();
  private static final String EXPECTED_GENERATED_STRING = "test";

  @Nested
  class Process {

    Template template;

    @BeforeEach
    void setUp() throws IOException {
      TemplateFactory templateFactory = new TemplateFactory(TEMPLATE_DIR);
      template = templateFactory.getTemplate(TEMPLATE);
    }

    @AfterEach
    void cleanUp() {
      FileUtils.delete(OUT_FILE);
    }

    @Test
    void shouldGenerateToFile() throws IOException, TemplateException {
      FileTemplatePrinter templatePrinter = new FileTemplatePrinter(template, OUT_FILE);

      templatePrinter.process(EMPTY_DATA_MODEL);

      TextFile generatedFile = new TextFile(OUT_FILE);
      assertThat(generatedFile.read()).isEqualTo(EXPECTED_GENERATED_STRING);
    }
  }

}