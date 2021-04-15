/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */

package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

public class EmbeddedMontiArcMathModelLoader extends de.monticore.modelloader.ModelingLanguageModelLoader<ASTEMACompilationUnit> {

  public EmbeddedMontiArcMathModelLoader(EmbeddedMontiArcMathLanguage language) {
    super(language);
  }

  @Override
  protected void createSymbolTableFromAST(final ASTEMACompilationUnit ast, final String modelName,
                                          final MutableScope enclosingScope, final ResolvingConfiguration resolvingConfiguration) {
    final EmbeddedMontiArcMathSymbolTableCreator symbolTableCreator =
            getModelingLanguage().getSymbolTableCreator(resolvingConfiguration, enclosingScope).orElse(null);

    if (symbolTableCreator != null) {
      Log.debug("Start creation of symbol table for model \"" + modelName + "\".",
          EmbeddedMontiArcMathModelLoader.class.getSimpleName());
      final Scope scope = symbolTableCreator.createFromAST(ast);

      if (!(scope instanceof ArtifactScope)) {
        Log.warn("0xA7001_184 Top scope of model " + modelName + " is expected to be an artifact scope, but"
          + " is scope \"" + scope.getName() + "\"");
      }

      Log.debug("Created symbol table for model \"" + modelName + "\".", EmbeddedMontiArcMathModelLoader.class.getSimpleName());
    }
    else {
      Log.warn("0xA7002_184 No symbol created, because '" + getModelingLanguage().getName()
        + "' does not define a symbol table creator.");
    }
  }

  @Override
  public EmbeddedMontiArcMathLanguage getModelingLanguage() {
    return (EmbeddedMontiArcMathLanguage) super.getModelingLanguage();
  }
}
