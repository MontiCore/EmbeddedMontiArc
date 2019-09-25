/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;
import java.util.stream.Collectors;

@Singleton
public class NodeModulesResolverImpl implements NodeModulesResolver {
    protected final NotificationService notifications;
    protected final NodeModulesFactory factory;

    @Inject
    protected NodeModulesResolverImpl(NotificationService notifications, NodeModulesFactory factory) {
        this.notifications = notifications;
        this.factory = factory;
    }

    protected void doResolve(File current, List<NodeModules> collection, boolean recursive) {
        File[] directories = Optional.ofNullable(current.listFiles(File::isDirectory)).orElse(new File[0]);
        Predicate<File> nameFilter = file -> file.getName().equals("node_modules");
        List<File> matches = Arrays.stream(directories).filter(nameFilter).collect(Collectors.toList());
        List<NodeModules> nodeModules = matches.stream().map(this.factory::create).collect(Collectors.toList());

        collection.addAll(nodeModules);

        if ((recursive || collection.size() == 0) && current.getParentFile() != null)
            this.doResolve(current.getParentFile(), collection, recursive);
    }

    @Override
    public List<NodeModules> resolve(File start) {
        List<NodeModules> nodeModules = new ArrayList<>();

        this.notifications.info("Resolving 'node_modules' starting from '%s'.", start);
        this.doResolve(start, nodeModules, true);

        return nodeModules;
    }

    @Override
    public Optional<NodeModules> resolveFirst(File start) {
        List<NodeModules> nodeModules = new ArrayList<>();

        this.notifications.info("Resolving first 'node_modules' starting from %s.", start);
        this.doResolve(start, nodeModules, false);

        return nodeModules.size() == 0 ? Optional.empty() : Optional.of(nodeModules.get(0));
    }
}
