/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.plugins.common.PluginsGenerateModule;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorSetupContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable.TemplateVariable;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.EnvironmentGeneratePlugin;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.configuration.EnvironmentGenerateConfiguration;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.configuration.EnvironmentGenerateConfigurationImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.*;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.collector.DDFCollector;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.collector.DDFCollectorImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.partitioner.DDFPartitioner;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.partitioner.DDFPartitionerImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.sanitizer.DDFSanitizer;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.sanitizer.DDFSanitizerImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.sanitizer.DDFSanitizerPhase;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.sanitizer.NoDuplicatePackage;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.translator.DDFTranslator;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.translator.DDFTranslatorImpl;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.configuration.EnvironmentValidateConfiguration;

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
        bind(EnvironmentValidateConfiguration.class).to(EnvironmentGenerateConfigurationImpl.class);
        bind(GeneratePluginConfiguration.class).to(EnvironmentGenerateConfigurationImpl.class);
        bind(DDFTranslator.class).to(DDFTranslatorImpl.class);
        bind(EnvironmentGeneratePlugin.class).toInstance(this.plugin);
        bind(DDFCollector.class).to(DDFCollectorImpl.class);
        bind(DDFPartitioner.class).to(DDFPartitionerImpl.class);
        bind(DDFSanitizer.class).to(DDFSanitizerImpl.class);
    }

    private void addMultiBindings() {
        this.addGeneratorPhases();
        this.addGlexContributions();
        this.addTemplateVariables();
        this.addTemplateContributions();
        this.addGeneratorSetupContributions();
        this.addSubSanitizers();
    }

    private void installModules() {
        this.install(new PluginsGenerateModule(this.plugin));
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
        Multibinder<DDFSanitizerPhase> contributions = Multibinder.newSetBinder(binder(), DDFSanitizerPhase.class);

        contributions.addBinding().to(NoDuplicatePackage.class);
    }
}
