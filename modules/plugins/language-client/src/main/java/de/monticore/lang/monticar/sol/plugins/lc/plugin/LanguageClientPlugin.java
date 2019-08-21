/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin;

import com.google.common.collect.ImmutableList;
import com.google.inject.Module;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.AbstractGeneratePlugin;
import de.monticore.lang.monticar.sol.plugins.lc.LanguageClientModule;
import org.apache.maven.plugins.annotations.LifecyclePhase;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.util.List;

import static com.google.common.base.MoreObjects.firstNonNull;

@Mojo(name = "generate", inheritByDefault = false, defaultPhase = LifecyclePhase.GENERATE_SOURCES)
public class LanguageClientPlugin extends AbstractGeneratePlugin {
    @Override
    public Module getModule() {
        return new LanguageClientModule(this);
    }

    @Parameter
    protected List<File> handCodedPaths;

    /**
     * @return The directories which hold handwritten code.
     */
    public List<File> getHandCodedPaths() {
        return firstNonNull(this.handCodedPaths, ImmutableList.of());
    }

    @Parameter(required = true)
    protected String rootModel;

    /**
     * @return The root model for the language description.
     */
    public String getRootModel() {
        return this.rootModel;
    }

    @Parameter(required = true)
    protected String grammar;

    /**
     * @return Qualified name of the grammar.
     */
    public String getGrammar() {
        return this.grammar;
    }

    @Parameter(required = true)
    protected String extension;

    /**
     * @return The file extension that the models will have.
     */
    public String getExtension() {
        return this.extension;
    }

    @Parameter()
    public List<String> excludedKeywords;

    /**
     * @return A list of keywords which should be excluded.
     */
    public List<String> getExcludedKeywords() {
        return this.excludedKeywords;
    }

    @Parameter(defaultValue = ".")
    protected File outputDirectory;

    /**
     * @return The output directory. (Default: .).
     */
    public File getOutputPath() {
        return this.outputDirectory;
    }

    @Parameter(required = true)
    protected String grammarModule;

    /**
     * @return The artifactId of the module where the grammar is located.
     */
    public String getGrammarModule() {
        return this.grammarModule;
    }

    @Parameter(required = true)
    protected String serverArtifact;

    /**
     * @return The artifactId and name of the executable JAR of the language server. Format: (artifactId:artifactName)
     */
    public String getServerArtifact() {
        return this.serverArtifact;
    }
}
