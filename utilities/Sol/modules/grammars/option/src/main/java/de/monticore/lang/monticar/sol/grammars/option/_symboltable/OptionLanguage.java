/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option._symboltable;

import com.google.inject.Singleton;
import de.monticore.ast.ASTNode;
import de.monticore.modelloader.ModelingLanguageModelLoader;

@Singleton
public class OptionLanguage extends OptionLanguageTOP {
    public OptionLanguage() {
        this("Option", "option");
    }

    protected OptionLanguage(String langName, String fileEnding) {
        super(langName, fileEnding);
    }

    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new OptionModelLoader(this);
    }
}
