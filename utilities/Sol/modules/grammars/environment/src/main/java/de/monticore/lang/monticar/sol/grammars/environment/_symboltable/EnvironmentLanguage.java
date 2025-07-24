/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment._symboltable;

import com.google.inject.Singleton;
import de.monticore.ast.ASTNode;
import de.monticore.modelloader.ModelingLanguageModelLoader;

@Singleton
public class EnvironmentLanguage extends EnvironmentLanguageTOP {
    public EnvironmentLanguage() {
        this("Environment Contribution", "ec");
    }

    protected EnvironmentLanguage(String langName, String fileEnding) {
        super(langName, fileEnding);
    }

    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new EnvironmentModelLoader(this);
    }
}
