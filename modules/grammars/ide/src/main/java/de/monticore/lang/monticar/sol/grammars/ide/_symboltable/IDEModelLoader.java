/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentLanguage;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTIDECompilationUnit;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionLanguage;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactLanguage;
import de.monticore.lang.monticar.sol.runtime.grammar.symboltable.ModelLoader;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Collection;

@Singleton
public class IDEModelLoader extends IDEModelLoaderTOP implements ModelLoader {
    protected final ResolvingConfiguration configuration;
    protected final ModelingLanguageFamily family;

    public IDEModelLoader(IDELanguage language) {
        super(language);

        this.family = this.initModelingLanguageFamily(language);
        this.configuration = this.initResolvingConfiguration(language);
    }

    @Inject
    protected IDEModelLoader(IDELanguage ide, ArtifactLanguage artifact, OptionLanguage option,
                             EnvironmentLanguage environment) {
        super(ide);

        this.family = this.initModelingLanguageFamily(ide, artifact, option, environment);
        this.configuration = this.initResolvingConfiguration(ide, artifact, option, environment);
    }

    public Collection<ASTIDECompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath, MutableScope enclosingScope) {
        return this.loadModelsIntoScope(qualifiedModelName, modelPath, enclosingScope, this.configuration);
    }

    public Collection<ASTIDECompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath) {
        GlobalScope scope = new GlobalScope(modelPath, this.family);

        return this.loadModelsIntoScope(qualifiedModelName, modelPath, scope);
    }
}
