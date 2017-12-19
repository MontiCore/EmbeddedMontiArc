package de.monticore.lang.monticar.resolver;

import de.monticore.lang.monticar.generator.cpp.resolver.Resolver;
import de.monticore.lang.monticar.generator.cpp.resolver.SymTabCreator;
import java.nio.file.Path;

public class ResolverFactory {

  private Path[] modelPaths;

  public ResolverFactory(Path... modelPaths) {
    this.modelPaths = modelPaths;
  }

  public Resolver get() {
    SymTabCreator symTabCreator = new SymTabCreator(modelPaths);
    return new Resolver(symTabCreator.createSymTab());
  }
}
