/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.AbstractPlugin;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationServiceImpl;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.hc.HandCodeServiceImpl;

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
    }

    private void addBindings() {
        bind(NotificationService.class).to(NotificationServiceImpl.class);
        bind(AbstractPlugin.class).toInstance(this.plugin);
    }

    private void addMultiBindings() {
        this.addPluginContributions();
    }

    private void addPluginContributions() {
        Multibinder<PluginContribution> contributions =
                Multibinder.newSetBinder(binder(), PluginContribution.class);

        contributions.addBinding().to(NotificationServiceImpl.class);
        contributions.addBinding().to(HandCodeServiceImpl.class);
    }

    @Provides
    private List<PluginContribution> provideSortedContributions(Set<PluginContribution> contributions) {
        Comparator<PluginContribution> comparator = Comparator.comparingInt(PluginContribution::getPriority);

        return contributions.stream().sorted(comparator.reversed()).collect(Collectors.toList());
    }
}
