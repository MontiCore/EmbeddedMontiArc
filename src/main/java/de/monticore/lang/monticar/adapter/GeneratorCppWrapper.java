/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.adapter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.util.TextFile;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

@Component
public class GeneratorCppWrapper {

  private GeneratorCPP generator;
  private TaggingResolver symtab;
  private Path modelPath;

  @Autowired
  public GeneratorCppWrapper(GeneratorCPP generator, TaggingResolver symtab, Path modelPath) {
    this.generator = generator;
    this.symtab = symtab;
    this.modelPath = modelPath;
  }

  public String getTargetPath() {
    return generator.getGenerationTargetPath();
  }

  public void setTargetPath(Path path) {
    generator.setGenerationTargetPath(path.toString());
  }

  public void setUseAlgebraicOptimization(boolean useAlgebraicOptimization) {
    generator.setUseAlgebraicOptimizations(useAlgebraicOptimization);
  }

  public void setGenerateTests(boolean generateTests) {
    generator.setGenerateTests(generateTests);
  }

  public TextFile generateFiles(ExpandedComponentInstanceSymbol componentSymbol)
      throws IOException {
    generator.useArmadilloBackend();
    generator.setModelsDirPath(modelPath);
    List<File> files = generator.generateFiles(symtab, componentSymbol, symtab);
    return new TextFile(files.get(0).toPath());
  }
}
