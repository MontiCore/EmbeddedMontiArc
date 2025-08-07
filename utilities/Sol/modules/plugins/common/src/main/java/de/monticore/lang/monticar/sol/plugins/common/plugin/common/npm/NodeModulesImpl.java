/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import com.google.inject.Inject;
import com.google.inject.assistedinject.Assisted;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.NameFileFilter;
import org.apache.commons.io.filefilter.TrueFileFilter;
import org.json.JSONException;

import java.io.File;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.Collectors;

public class NodeModulesImpl implements NodeModules {
    protected final File path;
    protected final NotificationService notifications;
    protected final NPMPackageFactory factory;
    protected final List<NPMPackage> packages;

    @Inject
    protected NodeModulesImpl(@Assisted File path, NotificationService notifications, NPMPackageFactory factory) {
        this.path = path;
        this.notifications = notifications;
        this.factory = factory;
        this.packages = this.fetchPackages(path);
    }

    protected List<NPMPackage> fetchPackages(File path) {
        this.notifications.info("Fetching NPMPackages for '%s'.", this.path);

        Function<File, Optional<NPMPackage>> mapping = file -> {
            try {
                return Optional.of(this.factory.create(file));
            } catch (JSONException exception) {
                return Optional.empty();
            }
        };

        return FileUtils.listFiles(path, new NameFileFilter("package.json"), TrueFileFilter.INSTANCE).stream()
                .map(mapping).filter(Optional::isPresent).map(Optional::get).collect(Collectors.toList());
    }

    public File getPath() {
        return this.path;
    }

    public List<NPMPackage> getPackages() {
        return Collections.unmodifiableList(this.packages);
    }
}
