/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.freemarker;

import freemarker.template.Template;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;

public class StringTemplatePrinter implements TemplateProcessor {

  private Template template;
  private Writer writer = new StringWriter();

  public StringTemplatePrinter(Template template) {
    this.template = template;
  }

  public String print(Object dataModel) throws IOException, TemplateException {
    process(dataModel);
    return writer.toString();
  }

  @Override
  public void process(Object dataModel) throws IOException, TemplateException {
    writer = new StringWriter();
    template.process(dataModel, writer);
  }

  @Override
  public String toString() {
    return writer.toString();
  }
}
