/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.freemarker;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;

import freemarker.template.Template;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Path;

public class FileTemplatePrinter implements TemplateProcessor {

  private final Path outFile;
  private Template template;

  public FileTemplatePrinter(Template template, Path outFile) {
    this.template = requiresNotNull(template);
    this.outFile = requiresNotNull(outFile);
  }

  @Override
  public void process(Object dataModel) throws IOException, TemplateException {
    Files.createDirectories(outFile.getParent());
    try (Writer writer = Files.newBufferedWriter(outFile)) {
      template.process(dataModel, writer);
    }
  }

}
