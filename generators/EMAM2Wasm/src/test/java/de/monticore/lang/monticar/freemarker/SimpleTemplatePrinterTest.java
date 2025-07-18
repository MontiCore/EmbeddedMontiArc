/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.freemarker;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import freemarker.template.Template;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.io.Writer;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class SimpleTemplatePrinterTest {

  private static final Map<String, Object> EMPTY_DATA_MODEL = new HashMap<>();

  @Nested
  class Process {

    @Test
    void shouldDelegateParameters() throws IOException, TemplateException {
      Template template = mock(Template.class);
      Writer writer = mock(Writer.class);
      SimpleTemplatePrinter templatePrinter = new SimpleTemplatePrinter(template, writer);

      templatePrinter.process(EMPTY_DATA_MODEL);

      verify(template).process(EMPTY_DATA_MODEL, writer);
    }
  }
}
