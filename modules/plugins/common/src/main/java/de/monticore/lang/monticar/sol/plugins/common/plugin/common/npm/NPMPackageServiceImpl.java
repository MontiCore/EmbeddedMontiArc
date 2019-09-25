/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import org.apache.maven.plugin.Mojo;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;
import java.util.stream.Collectors;

@Singleton
public class NPMPackageServiceImpl implements NPMPackageService, PluginContribution {
    protected final NotificationService notifications;
    protected final PluginConfiguration configuration;
    protected final NodeModulesResolver resolver;
    protected final NPMPackageFactory factory;
    protected final List<NPMPackage> packages;

    protected NPMPackage currentPackage;

    @Inject
    protected NPMPackageServiceImpl(NotificationService notifications, PluginConfiguration configuration,
                                    NodeModulesResolver resolver, NPMPackageFactory factory) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.resolver = resolver;
        this.factory = factory;
        this.packages = new ArrayList<>();
    }

    @Override
    public Optional<SolPackage> getCurrentPackage() {
        return Optional.of(this.currentPackage).filter(NPMPackage::isSolPackage).flatMap(NPMPackage::getAsSolPackage);
    }

    @Override
    public List<NPMPackage> getAllPackages() {
        return Collections.unmodifiableList(this.packages);
    }

    @Override
    public Optional<NPMPackage> resolve(String name) {
        Predicate<NPMPackage> predicate = p -> p.getName().isPresent() && p.getName().get().equals(name);

        return this.packages.stream().filter(predicate).findFirst();
    }

    @Override
    public List<NPMPackage> resolveMany(Predicate<NPMPackage> predicate) {
        return this.packages.stream().filter(predicate).collect(Collectors.toList());
    }

    @Override
    public int getPriority() {
        return 40000;
    }

    @Override
    public void onPluginConfigure(Mojo plugin) {
        File currentDirectory = this.configuration.getMavenProject().getBasedir();
        File packageFile = new File(currentDirectory, "package.json");

        if (packageFile.exists()) this.fetchPackages(currentDirectory, packageFile);
        else this.notifications.info("Plugin not operating on NPM environment. Skipping NPM package detection.");
    }

    protected void fetchPackages(File currentDirectory, File packageFile) {
        this.notifications.info("Plugin operating on NPM environment. Detecting NPM packages.");

        List<NodeModules> nodeModules = this.resolver.resolve(currentDirectory);
        List<NPMPackage> packages =
                nodeModules.stream().flatMap(modules -> modules.getPackages().stream()).collect(Collectors.toList());

        this.currentPackage = this.factory.create(packageFile);

        this.packages.addAll(packages);
        this.packages.add(this.currentPackage);
    }
}
