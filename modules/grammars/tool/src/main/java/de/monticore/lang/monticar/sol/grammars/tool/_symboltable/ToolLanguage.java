/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable;

import com.google.inject.Singleton;
import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileResolvingFilter;
import de.monticore.modelloader.ModelingLanguageModelLoader;

@Singleton
public class ToolLanguage extends ToolLanguageTOP {
    public ToolLanguage() {
        super("ToolDescription", "td");
    }

    protected ToolLanguage(String langName, String fileEnding) {
        super(langName, fileEnding);
    }

    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new ToolModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();

        this.addResolvingFilter(new DockerfileResolvingFilter());
    }
}
