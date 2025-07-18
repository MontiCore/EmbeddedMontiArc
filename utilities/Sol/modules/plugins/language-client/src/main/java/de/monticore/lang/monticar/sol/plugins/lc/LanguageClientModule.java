/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.language.LanguageModule;
import de.monticore.lang.monticar.sol.grammars.option.OptionModule;
import de.monticore.lang.monticar.sol.plugins.common.PluginsGenerateModule;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorSetupContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable.TemplateVariable;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.LanguageClientPlugin;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfigurationImpl;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.LanguageClientGeneratorSetup;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.LanguageClientGlex;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.lc.LCExtractor;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.lc.LCExtractorImpl;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.lc.LCGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.server.ServerGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.template.GrammarNameVariable;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.template.LanguageClientTemplates;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.template.TemplateGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.textmate.TextMateGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTableImpl;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.validator.LDValidator;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.validator.LDValidatorImpl;
import de.monticore.lang.monticar.sol.plugins.option.OptionPluginModule;

public class LanguageClientModule extends AbstractModule {
    private final LanguageClientPlugin plugin;

    public LanguageClientModule(LanguageClientPlugin plugin) {
        this.plugin = plugin;
    }

    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
        this.installModules();
    }

    private void addBindings() {
        bind(LanguageClientPlugin.class).toInstance(this.plugin);
        bind(LanguageClientConfiguration.class).to(LanguageClientConfigurationImpl.class);
        bind(PluginConfiguration.class).to(LanguageClientConfigurationImpl.class);
        bind(GeneratePluginConfiguration.class).to(LanguageClientConfigurationImpl.class);
        bind(LDValidator.class).to(LDValidatorImpl.class);
        bind(LCExtractor.class).to(LCExtractorImpl.class);
        bind(LanguageSymbolTable.class).to(LanguageSymbolTableImpl.class);
    }

    private void addMultiBindings() {
        this.addGeneratorPhases();
        this.addTemplateContributions();
        this.addTemplateVariables();
        this.addGlexContributions();
        this.addGeneratorSetupContributions();
        this.addPluginContributions();
    }

    private void installModules() {
        this.install(new PluginsGenerateModule(this.plugin));
        this.install(new LanguageModule());
        this.install(new OptionModule());
        this.install(new OptionPluginModule());
    }

    private void addGeneratorPhases() {
        Multibinder<GeneratorPhase> contributions = Multibinder.newSetBinder(binder(), GeneratorPhase.class);

        contributions.addBinding().to(TemplateGeneratorPhase.class);
        contributions.addBinding().to(TextMateGeneratorPhase.class);
        contributions.addBinding().to(ServerGeneratorPhase.class);
        contributions.addBinding().to(LCGeneratorPhase.class);
    }

    private void addTemplateContributions() {
        Multibinder<TemplateContribution> contributions = Multibinder.newSetBinder(binder(), TemplateContribution.class);

        contributions.addBinding().to(LanguageClientTemplates.class);
    }

    private void addTemplateVariables() {
        Multibinder<TemplateVariable> contributions = Multibinder.newSetBinder(binder(), TemplateVariable.class);

        contributions.addBinding().to(GrammarNameVariable.class);
    }

    private void addGlexContributions() {
        Multibinder<GlexContribution> contributions = Multibinder.newSetBinder(binder(), GlexContribution.class);

        contributions.addBinding().to(LanguageClientGlex.class);
    }

    private void addGeneratorSetupContributions() {
        Multibinder<GeneratorSetupContribution> contributions =
                Multibinder.newSetBinder(binder(), GeneratorSetupContribution.class);

        contributions.addBinding().to(LanguageClientGeneratorSetup.class);
    }

    private void addPluginContributions() {
        Multibinder<PluginContribution> contributions = Multibinder.newSetBinder(binder(), PluginContribution.class);

        contributions.addBinding().to(LDValidatorImpl.class);
        contributions.addBinding().to(LanguageSymbolTableImpl.class);
    }
}
