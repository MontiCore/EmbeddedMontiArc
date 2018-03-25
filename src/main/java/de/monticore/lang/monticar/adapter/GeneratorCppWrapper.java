package de.monticore.lang.monticar.adapter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.SymTabCreator;
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

  @Autowired
  public GeneratorCppWrapper(GeneratorCPP generator) {
    this.generator = generator;
  }

  public String getTargetPath() {
    return generator.getGenerationTargetPath();
  }

  public void setTargetPath(Path path) {
    generator.setGenerationTargetPath(path.toString());
  }

  public TextFile generateFiles(ExpandedComponentInstanceSymbol componentSymbol,
      TaggingResolver symtab) throws IOException {
    generator.useArmadilloBackend();
    generator.setUseAlgebraicOptimizations(true);
    List<File> files = generator.generateFiles(symtab, componentSymbol, symtab);
    return new TextFile(files.get(0).toPath());
  }

  public TextFile generateFiles(Path emaPath, String fullName) throws IOException {
    SymTabCreator symTabCreator = new SymTabCreator(emaPath);
    TaggingResolver symtab = symTabCreator.createSymTabAndTaggingResolver();
    Resolver resolver = new Resolver(symtab);
    ExpandedComponentInstanceSymbol componentSymbol = resolver
        .getExpandedComponentInstanceSymbol(fullName);

    return generateFiles(componentSymbol, symtab);
  }
}