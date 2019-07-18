/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.configuration;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.AbstractPluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.EnvironmentValidatePlugin;

import java.io.File;

@Singleton
public class EnvironmentValidateConfigurationImpl extends AbstractPluginConfiguration
        implements EnvironmentValidateConfiguration {
    protected final EnvironmentValidatePlugin plugin;

    @Inject
    protected EnvironmentValidateConfigurationImpl(EnvironmentValidatePlugin plugin) {
        super(plugin);

        this.plugin = plugin;
    }

    @Override
    public File getStatePath() {
        return new File(this.getMavenProject().getBasedir(), "state");
    }

    @Override
    public File getRootDirectory() {
        return this.plugin.getRootDirectory();
    }
}
