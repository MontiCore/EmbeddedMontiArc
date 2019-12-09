/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.environment.EnvironmentModule;
import de.monticore.lang.monticar.sol.grammars.ide.IDEModule;
import de.monticore.lang.monticar.sol.grammars.option.OptionModule;
import de.monticore.lang.monticar.sol.grammars.artifact.ArtifactModule;
import de.monticore.lang.monticar.sol.plugins.common.PluginsGenerateModule;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorSetupContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable.TemplateVariable;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.IDEPlugin;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.configuration.IDEConfiguration;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.configuration.IDEConfigurationImpl;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.*;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.filter.LocalFilter;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.filter.LocalFilterImpl;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer.*;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.order.OrderService;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.order.OrderServiceImpl;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.template.IDETemplates;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.template.TemplateGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.external.ExternalGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.external.resolver.SolExtensionsResolver;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.external.resolver.SolExtensionsResolverImpl;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.internal.InternalGeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.IDESymbolTable;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.IDESymbolTableImpl;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.validator.IDEValidator;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.validator.IDEValidatorImpl;
import de.monticore.lang.monticar.sol.plugins.option.OptionPluginModule;

public class IDEGenerateModule extends AbstractModule {
    private final IDEPlugin plugin;

    public IDEGenerateModule(IDEPlugin plugin) {
        this.plugin = plugin;
    }

    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
        this.installModules();
    }

    private void addBindings() {
        bind(IDEPlugin.class).toInstance(this.plugin);
        bind(IDEConfiguration.class).to(IDEConfigurationImpl.class);
        bind(PluginConfiguration.class).to(IDEConfigurationImpl.class);
        bind(GeneratePluginConfiguration.class).to(IDEConfigurationImpl.class);
        bind(IDESymbolTable.class).to(IDESymbolTableImpl.class);
        bind(IDEValidator.class).to(IDEValidatorImpl.class);
        bind(IDEMethodDelegator.class).to(IDEMethodDelegatorImpl.class);
        bind(IDEPrinter.class).to(IDEPrinterImpl.class);
        bind(OrderService.class).to(OrderServiceImpl.class);
        bind(NamePrinter.class).to(NamePrinterImpl.class);
        bind(ImportPrinter.class).to(ImportPrinterImpl.class);
        bind(SolExtensionsResolver.class).to(SolExtensionsResolverImpl.class);
        bind(LocalFilter.class).to(LocalFilterImpl.class);
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
        this.install(new OptionModule());
        this.install(new EnvironmentModule());
        this.install(new IDEModule());
        this.install(new OptionPluginModule());
    }

    private void addPluginContributions() {
        Multibinder<PluginContribution> contributions = Multibinder.newSetBinder(binder(), PluginContribution.class);

        contributions.addBinding().to(IDESymbolTableImpl.class);
        contributions.addBinding().to(IDEValidatorImpl.class);
    }

    private void addGlexContributions() {
        Multibinder<GlexContribution> contributions = Multibinder.newSetBinder(binder(), GlexContribution.class);

        contributions.addBinding().to(IDEGlex.class);
    }

    private void addGeneratorPhases() {
        Multibinder<GeneratorPhase> phases = Multibinder.newSetBinder(binder(), GeneratorPhase.class);

        phases.addBinding().to(TemplateGeneratorPhase.class);
        phases.addBinding().to(InternalGeneratorPhase.class);
        phases.addBinding().to(ExternalGeneratorPhase.class);
    }

    private void addTemplateContributions() {
        Multibinder<TemplateContribution> contributions =
                Multibinder.newSetBinder(binder(), TemplateContribution.class);

        contributions.addBinding().to(IDETemplates.class);
    }

    private void addGeneratorSetupContributions() {
        Multibinder<GeneratorSetupContribution> contributions =
                Multibinder.newSetBinder(binder(), GeneratorSetupContribution.class);

        contributions.addBinding().to(IDEGeneratorSetup.class);
    }

    private void addTemplateVariables() {
        Multibinder.newSetBinder(binder(), TemplateVariable.class);
    }
}
