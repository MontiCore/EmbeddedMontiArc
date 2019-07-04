/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.AbstractGeneratePlugin;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.Generator;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorImpl;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.hc.HandCodeService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.hc.HandCodeServiceImpl;

public class PluginsGenerateModule extends AbstractModule {
    private final AbstractGeneratePlugin plugin;

    public PluginsGenerateModule(AbstractGeneratePlugin plugin) {
        this.plugin = plugin;
    }

    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
        this.installModules();
    }

    private void addBindings() {
        bind(Generator.class).to(GeneratorImpl.class);
        bind(AbstractGeneratePlugin.class).toInstance(this.plugin);
        bind(HandCodeService.class).to(HandCodeServiceImpl.class);
    }

    private void addMultiBindings() {
        this.addPluginContributions();
    }

    private void installModules() {
        this.install(new PluginsCommonModule(this.plugin));
    }

    private void addPluginContributions() {
        Multibinder<PluginContribution> contributions = Multibinder.newSetBinder(binder(), PluginContribution.class);

        contributions.addBinding().to(GeneratorImpl.class);
        contributions.addBinding().to(HandCodeServiceImpl.class);
    }
}
