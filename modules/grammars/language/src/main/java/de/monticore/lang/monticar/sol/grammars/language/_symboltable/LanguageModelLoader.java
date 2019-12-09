/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionLanguage;
import de.monticore.lang.monticar.sol.runtime.grammar.symboltable.ModelLoader;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Collection;

@Singleton
public class LanguageModelLoader extends LanguageModelLoaderTOP implements ModelLoader {
    protected final ResolvingConfiguration configuration;
    protected final ModelingLanguageFamily family;

    public LanguageModelLoader(LanguageLanguage language) {
        super(language);

        this.configuration = this.initResolvingConfiguration(language);
        this.family = this.initModelingLanguageFamily(language);
    }

    @Inject
    protected LanguageModelLoader(LanguageLanguage language, OptionLanguage optionLanguage) {
        super(language);

        this.configuration = this.initResolvingConfiguration(language, optionLanguage);
        this.family = this.initModelingLanguageFamily(language, optionLanguage);
    }

    public Collection<ASTLanguageCompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath, MutableScope enclosingScope) {
        return this.loadModelsIntoScope(qualifiedModelName, modelPath, enclosingScope, this.configuration);
    }

    public Collection<ASTLanguageCompilationUnit> loadModelsIntoScope(String qualifiedModelName, ModelPath modelPath) {
        GlobalScope scope = new GlobalScope(modelPath, this.family);

        return this.loadModelsIntoScope(qualifiedModelName, modelPath, scope);
    }
}
