/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration;

import com.google.common.base.Preconditions;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.AbstractGeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.LanguageClientPlugin;

import java.io.File;
import java.util.List;

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
    public File getStatePath() {
        return new File(this.getOutputPath(), "state");
    }

    @Override
    public String getGrammarQualifiedName() {
        Preconditions.checkNotNull(this.plugin);

        return this.plugin.getGrammar();
    }

    @Override
    public String getGrammarName() {
        String qualifiedName = this.getGrammarQualifiedName();
        String[] qualifiedParts = qualifiedName.split("\\.");

        return qualifiedParts[qualifiedParts.length - 1];
    }

    @Override
    public String getFileExtension() {
        Preconditions.checkNotNull(this.plugin);

        String extension = this.plugin.getExtension();

        return extension.startsWith(".") ? extension.substring(1) : extension;
    }

    @Override
    public List<String> getExcludedKeywords() {
        return this.plugin.getExcludedKeywords();
    }

    @Override
    public File getSourceCodeOutputPath() {
        return new File(this.getOutputPath(), "src-gen");
    }
}
