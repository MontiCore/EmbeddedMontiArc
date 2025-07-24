/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact._symboltable;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentLanguage;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageLanguage;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTArtifactCompilationUnit;
import de.monticore.lang.monticar.sol.runtime.grammar.symboltable.ModelLoader;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Collection;

@Singleton
public class ArtifactModelLoader extends ArtifactModelLoaderTOP implements ModelLoader {
    protected final ResolvingConfiguration configuration;
    protected final ModelingLanguageFamily family;

    public ArtifactModelLoader(ArtifactLanguage language) {
        super(language);

        this.family = this.initModelingLanguageFamily(language);
        this.configuration = this.initResolvingConfiguration(language);
    }

    @Inject
    protected ArtifactModelLoader(ArtifactLanguage artifact, EnvironmentLanguage environment, LanguageLanguage language) {
        super(artifact);

        this.family = this.initModelingLanguageFamily(artifact, environment, language);
        this.configuration = this.initResolvingConfiguration(artifact, environment, language);
    }

    public Collection<ASTArtifactCompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath, MutableScope enclosingScope) {
        return this.loadModelsIntoScope(qualifiedModelName, modelPath, enclosingScope, this.configuration);
    }

    public Collection<ASTArtifactCompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath) {
        GlobalScope scope = new GlobalScope(modelPath, this.family);

        return this.loadModelsIntoScope(qualifiedModelName, modelPath, scope);
    }
}
