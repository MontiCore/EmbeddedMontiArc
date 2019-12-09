/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.environment.EnvironmentModule;
import de.monticore.lang.monticar.sol.plugins.common.PluginsGenerateModule;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorSetupContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable.TemplateVariable;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.EnvironmentGeneratePlugin;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration.EnvironmentGenerateConfiguration;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration.EnvironmentGenerateConfigurationImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.DockerfileGenerator;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.EnvironmentGeneratorSetup;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.EnvironmentGlex;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.collector.ECCollector;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.collector.ECCollectorImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.partitioner.ECPartitioner;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.partitioner.ECPartitionerImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.sanitizer.ECSanitizer;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.sanitizer.ECSanitizerImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.sanitizer.ECSanitizerPhase;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.sanitizer.NoDuplicatePackage;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.translator.ECTranslator;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.translator.ECTranslatorImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.symboltable.EnvironmentSymbolTable;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.symboltable.EnvironmentSymbolTableImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.validator.EnvironmentValidator;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.validator.EnvironmentValidatorImpl;

public class EnvironmentGenerateModule extends AbstractModule {
    protected final EnvironmentGeneratePlugin plugin;

    public EnvironmentGenerateModule(EnvironmentGeneratePlugin plugin) {
        this.plugin = plugin;
    }

    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
        this.installModules();
    }

    private void addBindings() {
        bind(EnvironmentGenerateConfiguration.class).to(EnvironmentGenerateConfigurationImpl.class);
        bind(GeneratePluginConfiguration.class).to(EnvironmentGenerateConfigurationImpl.class);
        bind(PluginConfiguration.class).to(EnvironmentGenerateConfigurationImpl.class);
        bind(ECTranslator.class).to(ECTranslatorImpl.class);
        bind(EnvironmentGeneratePlugin.class).toInstance(this.plugin);
        bind(ECCollector.class).to(ECCollectorImpl.class);
        bind(ECPartitioner.class).to(ECPartitionerImpl.class);
        bind(ECSanitizer.class).to(ECSanitizerImpl.class);
        bind(EnvironmentValidator.class).to(EnvironmentValidatorImpl.class);
        bind(EnvironmentSymbolTable.class).to(EnvironmentSymbolTableImpl.class);
    }

    private void addMultiBindings() {
        this.addPluginContributions();
        this.addGeneratorPhases();
        this.addGlexContributions();
        this.addTemplateVariables();
        this.addTemplateContributions();
        this.addGeneratorSetupContributions();
        this.addSubSanitizers();
    }

    private void installModules() {
        this.install(new EnvironmentModule());
        this.install(new PluginsGenerateModule(this.plugin));
    }

    private void addPluginContributions() {
        Multibinder<PluginContribution> contributions = Multibinder.newSetBinder(binder(), PluginContribution.class);

        contributions.addBinding().to(EnvironmentValidatorImpl.class);
        contributions.addBinding().to(EnvironmentSymbolTableImpl.class);
    }

    private void addGeneratorPhases() {
        Multibinder<GeneratorPhase> contributions = Multibinder.newSetBinder(binder(), GeneratorPhase.class);

        contributions.addBinding().to(DockerfileGenerator.class);
    }

    private void addGlexContributions() {
        Multibinder<GlexContribution> contributions = Multibinder.newSetBinder(binder(), GlexContribution.class);

        contributions.addBinding().to(EnvironmentGlex.class);
    }

    private void addGeneratorSetupContributions() {
        Multibinder<GeneratorSetupContribution> contributions =
                Multibinder.newSetBinder(binder(), GeneratorSetupContribution.class);

        contributions.addBinding().to(EnvironmentGeneratorSetup.class);
    }

    private void addTemplateContributions() {
        Multibinder.newSetBinder(binder(), TemplateContribution.class);
    }

    private void addTemplateVariables() {
        Multibinder.newSetBinder(binder(), TemplateVariable.class);
    }

    private void addSubSanitizers() {
        Multibinder<ECSanitizerPhase> contributions = Multibinder.newSetBinder(binder(), ECSanitizerPhase.class);

        contributions.addBinding().to(NoDuplicatePackage.class);
    }
}
