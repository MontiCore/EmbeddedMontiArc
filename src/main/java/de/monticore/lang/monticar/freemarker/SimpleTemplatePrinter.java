/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.freemarker;

import freemarker.template.Template;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.io.Writer;

public class SimpleTemplatePrinter implements TemplateProcessor {

  private Template template;
  private Writer writer;

  public SimpleTemplatePrinter(Template template, Writer writer) {
    this.template = template;
    this.writer = writer;
  }

  @Override
  public void process(Object dataModel) throws IOException, TemplateException {
    template.process(dataModel, writer);
  }
}
