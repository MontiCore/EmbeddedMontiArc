/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTEMVCompilationUnit;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.EmbeddedMontiViewModelLoaderTOP;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Strings.isNullOrEmpty;

/**
 * Created by Michael von Wenckstern on 30.05.2016.
 *
 */
public class EmbeddedMontiViewModelLoader
    extends EmbeddedMontiViewModelLoaderTOP {

  public EmbeddedMontiViewModelLoader(EmbeddedMontiViewLanguage language) {
    super(language);
  }

  /**
   * this method should be implemented into the ModelPath class
   */
  private static Collection<Path> getEntriesFromModelPath(ModelPath modelPath) {
    String s = modelPath.toString().replace("[", "").replace("]", "").replace(" ", "");
    String ss[] = s.split(",");
    return Arrays.stream(ss).map(str -> Paths.get(URI.create(str))).collect(Collectors.toSet());
  }

  @Override
  public Collection<ASTEMVCompilationUnit> loadModelsIntoScope(final String qualifiedModelName, final ModelPath modelPath, final MutableScope enclosingScope, final ResolvingConfiguration ResolvingConfiguration) {

    final Collection<ASTEMVCompilationUnit> asts = loadModels(qualifiedModelName, modelPath);

    for (ASTEMVCompilationUnit ast : asts) {
      createSymbolTableFromAST(ast, qualifiedModelName, enclosingScope, ResolvingConfiguration);
    }

    return asts;
  }
}
