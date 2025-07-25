/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact._symboltable;

import com.google.inject.Singleton;
import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileResolvingFilter;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageResolvingFilter;
import de.monticore.modelloader.ModelingLanguageModelLoader;

@Singleton
public class ArtifactLanguage extends ArtifactLanguageTOP {
    public ArtifactLanguage() {
        super("Artifact Contribution", "ac");
    }

    protected ArtifactLanguage(String langName, String fileEnding) {
        super(langName, fileEnding);
    }

    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new ArtifactModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();

        this.addResolvingFilter(new DockerfileResolvingFilter());
        this.addResolvingFilter(new LanguageResolvingFilter());
    }
}
