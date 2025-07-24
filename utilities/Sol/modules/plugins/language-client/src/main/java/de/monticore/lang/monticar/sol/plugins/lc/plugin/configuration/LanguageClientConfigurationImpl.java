/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration;

import com.google.common.base.Preconditions;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.AbstractGeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.LanguageClientPlugin;

import java.io.File;

@Singleton
public class LanguageClientConfigurationImpl extends AbstractGeneratePluginConfiguration
        implements LanguageClientConfiguration {
    private final LanguageClientPlugin plugin;

    @Inject
    protected LanguageClientConfigurationImpl(LanguageClientPlugin plugin) {
        super(plugin);

        this.plugin = plugin;
    }

    @Override
    protected File getDefaultHandCodedPath() {
        return this.resolveFromBaseDirectory("src");
    }

    @Override
    public String getGrammarQualifiedName() {
        Preconditions.checkNotNull(this.plugin);

        String grammar = this.plugin.getGrammar();

        return grammar == null ? this.plugin.getRootModel() : grammar;
    }

    @Override
    public String getGrammarName() {
        String qualifiedName = this.getGrammarQualifiedName();
        String[] qualifiedParts = qualifiedName.split("\\.");

        return qualifiedParts[qualifiedParts.length - 1];
    }

    @Override
    public File getSourceCodeOutputPath() {
        return new File(this.getOutputPath(), "src-gen");
    }

    @Override
    public String getRootModel() {
        Preconditions.checkNotNull(this.plugin);

        return this.plugin.getRootModel();
    }
}
