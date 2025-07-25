/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin;

import com.google.common.collect.ImmutableList;
import com.google.inject.Module;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.AbstractGeneratePlugin;
import de.monticore.lang.monticar.sol.plugins.ls.LanguageServerPluginModule;
import org.apache.maven.plugins.annotations.LifecyclePhase;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.util.List;

import static com.google.common.base.MoreObjects.firstNonNull;

@Mojo(name = "generate", inheritByDefault = false, defaultPhase = LifecyclePhase.GENERATE_SOURCES)
public class LanguageServerPlugin extends AbstractGeneratePlugin {
    @Override
    public Module getModule() {
        return new LanguageServerPluginModule(this);
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
    protected String grammar;

    /**
     * @return Qualified name of the grammar.
     */
    public String getGrammar() {
        return this.grammar;
    }

    @Parameter(defaultValue = "${project.build.directory}/generated-sources/language-server")
    protected File outputDirectory;

    /**
     * @return The output directory. (Default: ${project.build.directory}/generated-sources/language-server).
     */
    public File getOutputPath() {
        return this.outputDirectory;
    }
}
