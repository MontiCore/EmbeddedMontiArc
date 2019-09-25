/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.assistedinject.FactoryModuleBuilder;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.AbstractGeneratePlugin;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.Generator;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorImpl;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.hc.HandCodeRegistry;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.hc.HandCodeRegistryImpl;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.*;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable.*;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

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
        bind(HandCodeRegistry.class).to(HandCodeRegistryImpl.class);
        bind(TemplateRegistry.class).to(TemplateRegistryImpl.class);
        bind(TemplateVariableService.class).to(TemplateVariableServiceImpl.class);
    }

    private void addMultiBindings() {
        this.addPluginContributions();
    }

    private void installModules() {
        this.install(new PluginsCommonModule(this.plugin));
        this.install(new FactoryModuleBuilder().implement(Template.class, TemplateImpl.class).build(TemplateFactory.class));
    }

    private void addPluginContributions() {
        Multibinder<PluginContribution> contributions = Multibinder.newSetBinder(binder(), PluginContribution.class);

        contributions.addBinding().to(GeneratorImpl.class);
        contributions.addBinding().to(HandCodeRegistryImpl.class);
        contributions.addBinding().to(TemplateRegistryImpl.class);
    }

    @Provides
    protected List<GeneratorPhase> provideSortedGeneratorPhases(Set<GeneratorPhase> phases) {
        Comparator<GeneratorPhase> comparator = Comparator.comparingInt(GeneratorPhase::getPriority);
        List<GeneratorPhase> sortedPhases = phases.stream().sorted(comparator).collect(Collectors.toList());

        Collections.reverse(sortedPhases);

        return sortedPhases;
    }
}
