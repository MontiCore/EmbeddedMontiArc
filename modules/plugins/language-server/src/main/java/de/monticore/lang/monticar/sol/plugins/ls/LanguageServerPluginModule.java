/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.ls;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.plugins.common.PluginsGenerateModule;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorSetupContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template.TemplateContribution;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.LanguageServerPlugin;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfigurationImpl;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.generator.LanguageServerGeneratorSetup;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.generator.LanguageServerGlex;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.generator.LanguageServerTemplateGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.template.LanguageModuleTemplate;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.template.ls.LanguageServerLauncherTemplate;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.template.services.LanguageDiagnosticsServiceTemplate;

public class LanguageServerPluginModule extends AbstractModule {
    private final LanguageServerPlugin plugin;

    public LanguageServerPluginModule(LanguageServerPlugin plugin) {
        this.plugin = plugin;
    }

    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
        this.installModules();
    }

    private void addBindings() {
        bind(PluginConfiguration.class).to(LanguageServerConfigurationImpl.class);
        bind(GeneratePluginConfiguration.class).to(LanguageServerConfigurationImpl.class);
        bind(LanguageServerConfiguration.class).to(LanguageServerConfigurationImpl.class);
        bind(LanguageServerPlugin.class).toInstance(this.plugin);
    }

    private void addMultiBindings() {
        this.addGeneratorPhases();
        this.addPluginContributions();
        this.addTemplateContributions();
        this.addGlexContributions();
        this.addGeneratorSetupContributions();
    }

    private void installModules() {
        this.install(new PluginsGenerateModule(this.plugin));
    }

    private void addPluginContributions() {
        Multibinder<PluginContribution> contributions =
                Multibinder.newSetBinder(binder(), PluginContribution.class);

        contributions.addBinding().to(LanguageServerConfigurationImpl.class);
    }

    private void addTemplateContributions() {
        Multibinder<TemplateContribution> contributions =
                Multibinder.newSetBinder(binder(), TemplateContribution.class);

        contributions.addBinding().to(LanguageServerLauncherTemplate.class);
        contributions.addBinding().to(LanguageDiagnosticsServiceTemplate.class);
        contributions.addBinding().to(LanguageModuleTemplate.class);
    }

    private void addGeneratorPhases() {
        Multibinder<GeneratorPhase> contributions =
                Multibinder.newSetBinder(binder(), GeneratorPhase.class);

        contributions.addBinding().to(LanguageServerTemplateGeneratorPhase.class);
    }

    private void addGlexContributions() {
        Multibinder<GlexContribution> contributions =
                Multibinder.newSetBinder(binder(), GlexContribution.class);

        contributions.addBinding().to(LanguageServerGlex.class);
    }

    private void addGeneratorSetupContributions() {
        Multibinder<GeneratorSetupContribution> contributions =
                Multibinder.newSetBinder(binder(), GeneratorSetupContribution.class);

        contributions.addBinding().to(LanguageServerGeneratorSetup.class);
    }
}
