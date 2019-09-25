/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration;

import com.google.common.base.Preconditions;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.AbstractPluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.AbstractGeneratePlugin;

import java.io.File;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public abstract class AbstractGeneratePluginConfiguration extends AbstractPluginConfiguration
        implements GeneratePluginConfiguration, PluginContribution {
    private final AbstractGeneratePlugin plugin;

    protected AbstractGeneratePluginConfiguration(AbstractGeneratePlugin plugin) {
        super(plugin);

        this.plugin = plugin;

        this.getMavenProject().addCompileSourceRoot(this.getSourceCodeOutputPath().getPath());
    }

    @Override
    public File getOutputPath() {
        Preconditions.checkNotNull(this.plugin);

        return this.resolveFromBaseDirectory(this.plugin.getOutputPath());
    }

    @Override
    public List<File> getHandCodedPaths() {
        Preconditions.checkNotNull(this.plugin);

        List<File> handCodedPaths = this.plugin.getHandCodedPaths();

        if (handCodedPaths.size() > 0)
            return handCodedPaths.stream().map(this::resolveFromBaseDirectory).collect(Collectors.toList());
        else
            return Collections.singletonList(this.getDefaultHandCodedPath());
    }

    protected abstract File getDefaultHandCodedPath();
}
