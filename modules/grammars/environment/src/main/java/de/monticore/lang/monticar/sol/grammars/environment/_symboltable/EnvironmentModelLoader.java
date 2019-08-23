/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.environment._symboltable;

import com.google.common.base.Preconditions;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnvironmentCompilationUnit;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;

@Singleton
public class EnvironmentModelLoader extends EnvironmentModelLoaderTOP {
    protected final ResolvingConfiguration configuration;

    public EnvironmentModelLoader(EnvironmentLanguage language) {
        super(language);

        this.configuration = null;
    }

    @Inject
    protected EnvironmentModelLoader(EnvironmentLanguage language, ResolvingConfiguration configuration) {
        super(language);

        this.configuration = configuration;
    }

    public Optional<ResolvingConfiguration> getConfiguration() {
        return Optional.ofNullable(this.configuration);
    }

    public Collection<ASTEnvironmentCompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath, MutableScope enclosingScope) {
        Preconditions.checkNotNull(this.configuration);

        return this.getConfiguration()
                .map(configuration -> this.loadModelsIntoScope(qualifiedModelName, modelPath, enclosingScope, configuration))
                .orElse(new ArrayList<>());
    }

    public Collection<ASTEnvironmentCompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath) {
        GlobalScope scope = new GlobalScope(modelPath, this.getModelingLanguage());

        return this.loadModelsIntoScope(qualifiedModelName, modelPath, scope);
    }
}
