/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration;

import com.google.common.base.Preconditions;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.AbstractGeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.LanguageServerPlugin;

import java.io.File;

@Singleton
public class LanguageServerConfigurationImpl extends AbstractGeneratePluginConfiguration
        implements LanguageServerConfiguration, PluginContribution {
    private final LanguageServerPlugin plugin;

    @Inject
    public LanguageServerConfigurationImpl(LanguageServerPlugin plugin) {
        super(plugin);

        this.plugin = plugin;
    }

    @Override
    protected File getDefaultHandCodedPath() {
        return this.resolveFromBaseDirectory("src/main/java");
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
    public String getGrammarGeneratedPackage() {
        return this.getGrammarQualifiedName().toLowerCase();
    }

    @Override
    public File getSourceCodeOutputPath() {
        return new File(this.getOutputPath(), "sourcecode");
    }

    @Override
    public String getPackageStructure() {
        return this.getGrammarGeneratedPackage().replaceAll("\\.", "/");
    }
}
