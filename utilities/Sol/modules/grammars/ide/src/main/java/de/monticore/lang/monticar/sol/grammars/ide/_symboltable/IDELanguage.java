/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import com.google.inject.Singleton;
import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactResolvingFilter;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ProductResolvingFilter;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolResolvingFilter;
import de.monticore.modelloader.ModelingLanguageModelLoader;

@Singleton
public class IDELanguage extends IDELanguageTOP {
    public IDELanguage() {
        super("IDE", "ide");
    }

    protected IDELanguage(String langName, String fileEnding) {
        super(langName, fileEnding);
    }

    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new IDEModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();

        this.addResolvingFilter(new ToolResolvingFilter());
        this.addResolvingFilter(new ProductResolvingFilter());
        this.addResolvingFilter(new ArtifactResolvingFilter());
    }
}
