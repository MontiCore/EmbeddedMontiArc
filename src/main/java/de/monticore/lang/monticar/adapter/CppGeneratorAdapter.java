/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.adapter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.freemarker.FileTemplatePrinter;
import de.monticore.lang.monticar.generator.cpp.CppGenerator;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.nio.file.Path;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Component;

@Component
public class CppGeneratorAdapter implements CppFileGenerator {

  private Template template;

  @Autowired
  public CppGeneratorAdapter(@Qualifier("cppTemplate") Template template) {
    this.template = template;
  }

  @Override
  public Path generate(ExpandedComponentInstanceSymbol model, Path outFile)
      throws IOException, TemplateException {
    CppGenerator generator = new CppGenerator(new FileTemplatePrinter(template, outFile));
    generator.generate(model);
    return outFile;
  }
}
