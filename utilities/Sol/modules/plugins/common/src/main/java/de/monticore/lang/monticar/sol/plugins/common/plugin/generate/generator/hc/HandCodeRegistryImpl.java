/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.hc;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;
import org.apache.commons.io.FileUtils;
import org.apache.maven.plugin.Mojo;

import java.io.File;
import java.nio.file.Path;
import java.util.*;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.UnaryOperator;
import java.util.stream.Collectors;

@Singleton
public class HandCodeRegistryImpl implements HandCodeRegistry, PluginContribution {
    protected final NotificationService notifications;
    protected final GeneratePluginConfiguration configuration;
    protected final Set<Path> handCodes;

    @Inject
    public HandCodeRegistryImpl(NotificationService notifications, GeneratePluginConfiguration configuration) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.handCodes = new HashSet<>();
    }

    @Override
    public Set<Path> getHandCodes() {
        return this.handCodes;
    }

    @Override
    public int getPriority() {
        return 100;
    }

    @Override
    public void onPluginConfigure(Mojo plugin) {
        Predicate<File> predicate = handCodedPath -> handCodedPath.exists() && handCodedPath.isDirectory();
        Function<File, Collection<Path>> mapping = handCodedPath -> {
            Collection<File> handCodes = FileUtils.listFiles(handCodedPath, null, true);
            UnaryOperator<Path> relativize = path -> handCodedPath.toPath().relativize(path);

            return handCodes.stream().map(File::toPath).map(relativize).collect(Collectors.toList());
        };

        this.notifications.info("Detecting Handwritten Code.");
        this.configuration.getHandCodedPaths().stream().filter(predicate).map(mapping).forEach(this.handCodes::addAll);
    }
}
