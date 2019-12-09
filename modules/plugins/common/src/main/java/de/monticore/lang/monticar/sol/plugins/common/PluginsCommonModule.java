/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.assistedinject.FactoryModuleBuilder;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.AbstractPlugin;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp.ModelPathService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp.ModelPathServiceImpl;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationServiceImpl;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.*;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.path.PathResolver;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.path.PathResolverImpl;

import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class PluginsCommonModule extends AbstractModule {
    private final AbstractPlugin plugin;

    public PluginsCommonModule(AbstractPlugin plugin) {
        this.plugin = plugin;
    }

    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
        this.installModules();
    }

    private void addBindings() {
        bind(NotificationService.class).to(NotificationServiceImpl.class);
        bind(NodeModulesResolver.class).to(NodeModulesResolverImpl.class);
        bind(NPMPackageService.class).to(NPMPackageServiceImpl.class);
        bind(ModelPathService.class).to(ModelPathServiceImpl.class);
        bind(PathResolver.class).to(PathResolverImpl.class);
        bind(AbstractPlugin.class).toInstance(this.plugin);
    }

    private void addMultiBindings() {
        this.addPluginContributions();
    }

    private void addPluginContributions() {
        Multibinder<PluginContribution> contributions =
                Multibinder.newSetBinder(binder(), PluginContribution.class);

        contributions.addBinding().to(NotificationServiceImpl.class);
        contributions.addBinding().to(NPMPackageServiceImpl.class);
    }

    private void installModules() {
        this.install(new FactoryModuleBuilder().implement(NodeModules.class, NodeModulesImpl.class).build(NodeModulesFactory.class));
        this.install(new FactoryModuleBuilder().implement(NPMPackage.class, NPMPackageImpl.class).build(NPMPackageFactory.class));
    }

    @Provides
    private List<PluginContribution> provideSortedContributions(Set<PluginContribution> contributions) {
        Comparator<PluginContribution> comparator = Comparator.comparingInt(PluginContribution::getPriority);

        return contributions.stream().sorted(comparator.reversed()).collect(Collectors.toList());
    }
}
