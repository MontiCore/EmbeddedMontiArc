/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment._symboltable;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnvironmentCompilationUnit;
import de.monticore.lang.monticar.sol.runtime.grammar.symboltable.ModelLoader;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Collection;

@Singleton
public class EnvironmentModelLoader extends EnvironmentModelLoaderTOP implements ModelLoader {
    protected final ResolvingConfiguration configuration;

    @Inject
    public EnvironmentModelLoader(EnvironmentLanguage language) {
        super(language);

        this.configuration = this.initResolvingConfiguration(language);
    }

    public Collection<ASTEnvironmentCompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath, MutableScope enclosingScope) {
        return this.loadModelsIntoScope(qualifiedModelName, modelPath, enclosingScope, this.configuration);
    }

    public Collection<ASTEnvironmentCompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath) {
        GlobalScope scope = new GlobalScope(modelPath, this.getModelingLanguage());

        return this.loadModelsIntoScope(qualifiedModelName, modelPath, scope);
    }
}
