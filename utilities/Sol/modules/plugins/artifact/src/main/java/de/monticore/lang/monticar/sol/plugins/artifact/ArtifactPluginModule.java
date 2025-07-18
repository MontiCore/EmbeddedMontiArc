/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.artifact;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.artifact.ArtifactModule;
import de.monticore.lang.monticar.sol.grammars.language.LanguageModule;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.ArtifactPlugin;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.configuration.ArtifactConfiguration;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.configuration.ArtifactConfigurationImpl;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.generator.ArtifactGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.generator.ArtifactGeneratorSetup;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.generator.template.ArtifactTemplates;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.generator.template.TemplateGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.symboltable.ArtifactSymbolTable;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.symboltable.ArtifactSymbolTableImpl;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.validator.ArtifactValidator;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.validator.ArtifactValidatorImpl;
import de.monticore.lang.monticar.sol.plugins.common.PluginsGenerateModule;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorSetupContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable.TemplateVariable;

public class ArtifactPluginModule extends AbstractModule {
    private final ArtifactPlugin plugin;

    public ArtifactPluginModule(ArtifactPlugin plugin) {
        this.plugin = plugin;
    }

    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
        this.installModules();
    }

    private void addBindings() {
        bind(ArtifactPlugin.class).toInstance(this.plugin);
        bind(ArtifactConfiguration.class).to(ArtifactConfigurationImpl.class);
        bind(PluginConfiguration.class).to(ArtifactConfigurationImpl.class);
        bind(GeneratePluginConfiguration.class).to(ArtifactConfigurationImpl.class);
        bind(ArtifactSymbolTable.class).to(ArtifactSymbolTableImpl.class);
        bind(ArtifactValidator.class).to(ArtifactValidatorImpl.class);
    }

    private void addMultiBindings() {
        this.addGlexContributions();
        this.addPluginContributions();
        this.addGeneratorPhases();
        this.addTemplateContributions();
        this.addTemplateVariables();
        this.addGeneratorSetupContributions();
    }

    private void installModules() {
        this.install(new PluginsGenerateModule(this.plugin));
        this.install(new ArtifactModule());
        this.install(new LanguageModule());
    }

    private void addPluginContributions() {
        Multibinder<PluginContribution> contributions = Multibinder.newSetBinder(binder(), PluginContribution.class);

        contributions.addBinding().to(ArtifactSymbolTableImpl.class);
        contributions.addBinding().to(ArtifactValidatorImpl.class);
    }

    private void addGlexContributions() {
        Multibinder.newSetBinder(binder(), GlexContribution.class);
    }

    private void addGeneratorPhases() {
        Multibinder<GeneratorPhase> phases = Multibinder.newSetBinder(binder(), GeneratorPhase.class);

        phases.addBinding().to(TemplateGeneratorPhase.class);
        phases.addBinding().to(ArtifactGeneratorPhase.class);
    }

    private void addTemplateContributions() {
        Multibinder<TemplateContribution> contributions =
                Multibinder.newSetBinder(binder(), TemplateContribution.class);

        contributions.addBinding().to(ArtifactTemplates.class);
    }

    private void addGeneratorSetupContributions() {
        Multibinder<GeneratorSetupContribution> contributions =
                Multibinder.newSetBinder(binder(), GeneratorSetupContribution.class);

        contributions.addBinding().to(ArtifactGeneratorSetup.class);
    }

    private void addTemplateVariables() {
        Multibinder.newSetBinder(binder(), TemplateVariable.class);
    }
}
