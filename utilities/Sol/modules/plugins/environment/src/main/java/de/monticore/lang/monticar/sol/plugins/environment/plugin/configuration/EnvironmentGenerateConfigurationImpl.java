/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.AbstractGeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.EnvironmentGeneratePlugin;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@Singleton
public class EnvironmentGenerateConfigurationImpl extends AbstractGeneratePluginConfiguration
        implements EnvironmentGenerateConfiguration {
    protected final EnvironmentGeneratePlugin plugin;

    @Inject
    protected EnvironmentGenerateConfigurationImpl(EnvironmentGeneratePlugin plugin) {
        super(plugin);

        this.plugin = plugin;
    }

    @Override
    public String getRootModel() {
        return this.plugin.getRootModel();
    }

    @Override
    public List<File> getHandCodedPaths() {
        return new ArrayList<>();
    }

    @Override
    public File getSourceCodeOutputPath() {
        return this.getOutputPath();
    }

    @Override
    protected File getDefaultHandCodedPath() {
        return null;
    }
}
