/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.adapter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.nio.file.Path;

public interface CppFileGenerator {

  Path generate(ExpandedComponentInstanceSymbol model, Path outFile)
      throws IOException, TemplateException;
}
