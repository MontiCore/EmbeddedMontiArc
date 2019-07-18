/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.environment.EnvironmentModule;
import de.monticore.lang.monticar.sol.plugins.common.PluginsCommonModule;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.EnvironmentValidatePlugin;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.configuration.EnvironmentValidateConfiguration;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.configuration.EnvironmentValidateConfigurationImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.validator.EnvironmentValidator;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.validator.EnvironmentValidatorImpl;

public class EnvironmentValidateModule extends AbstractModule {
    protected final EnvironmentValidatePlugin plugin;

    public EnvironmentValidateModule(EnvironmentValidatePlugin plugin) {
        this.plugin = plugin;
    }

    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
        this.installModules();
    }

    private void addBindings() {
        bind(EnvironmentValidatePlugin.class).toInstance(this.plugin);
        bind(EnvironmentValidateConfiguration.class).to(EnvironmentValidateConfigurationImpl.class);
        bind(EnvironmentValidator.class).to(EnvironmentValidatorImpl.class);
    }

    private void addMultiBindings() {
        this.addPluginContributions();
    }

    private void installModules() {
        this.install(new EnvironmentModule());
        this.install(new PluginsCommonModule(this.plugin));
    }

    private void addPluginContributions() {
        Multibinder<PluginContribution> contributions = Multibinder.newSetBinder(binder(), PluginContribution.class);

        contributions.addBinding().to(EnvironmentValidateConfigurationImpl.class);
        contributions.addBinding().to(EnvironmentValidatorImpl.class);
    }
}
