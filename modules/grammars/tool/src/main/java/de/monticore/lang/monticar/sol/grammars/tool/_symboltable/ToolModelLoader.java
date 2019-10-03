/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable;

import com.google.common.base.Preconditions;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.tool._ast.ASTToolCompilationUnit;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;

@Singleton
public class ToolModelLoader extends ToolModelLoaderTOP {
    protected final ResolvingConfiguration configuration;
    protected final ModelingLanguageFamily family;

    public ToolModelLoader(ToolLanguage language) {
        super(language);

        this.family = null;
        this.configuration = null;
    }

    @Inject
    protected ToolModelLoader(ToolLanguage language, ModelingLanguageFamily family, ResolvingConfiguration configuration) {
        super(language);

        this.family = family;
        this.configuration = configuration;
    }

    public Optional<ResolvingConfiguration> getConfiguration() {
        return Optional.ofNullable(this.configuration);
    }

    public Optional<ModelingLanguageFamily> getModelingLanguageFamily() {
        return Optional.ofNullable(this.family);
    }

    public Collection<ASTToolCompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath, MutableScope enclosingScope) {
        Preconditions.checkNotNull(this.configuration);

        return this.getConfiguration()
                .map(configuration -> this.loadModelsIntoScope(qualifiedModelName, modelPath, enclosingScope, configuration))
                .orElse(new ArrayList<>());
    }

    public Collection<ASTToolCompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath) {
        return this.getModelingLanguageFamily().map(family -> {
            GlobalScope scope = new GlobalScope(modelPath, family);

            return this.loadModelsIntoScope(qualifiedModelName, modelPath, scope);
        }).orElse(new ArrayList<>());
    }
}
