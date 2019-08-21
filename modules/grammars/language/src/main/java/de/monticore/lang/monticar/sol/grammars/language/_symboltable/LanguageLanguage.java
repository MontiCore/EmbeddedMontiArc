/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

import com.google.inject.Singleton;
import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.sol.grammars.options._symboltable.OptionPropResolvingFilter;
import de.monticore.lang.monticar.sol.grammars.options._symboltable.OptionResolvingFilter;
import de.monticore.modelloader.ModelingLanguageModelLoader;

@Singleton
public class LanguageLanguage extends LanguageLanguageTOP {
    public LanguageLanguage() {
        this("LanguageDescription", "ld");
    }

    protected LanguageLanguage(String langName, String fileEnding) {
        super(langName, fileEnding);
    }

    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new LanguageModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();

        this.addResolvingFilter(new OptionPropResolvingFilter());
        this.addResolvingFilter(new OptionResolvingFilter());
    }
}
