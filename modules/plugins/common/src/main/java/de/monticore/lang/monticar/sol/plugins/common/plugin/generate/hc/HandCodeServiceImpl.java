/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.hc;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;
import org.apache.commons.io.FileUtils;
import org.apache.maven.plugin.Mojo;

import java.io.File;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;

@Singleton
public class HandCodeServiceImpl implements HandCodeService, PluginContribution {
    protected final NotificationService notifications;
    protected final GeneratePluginConfiguration configuration;
    protected final List<Path> handCodes;

    @Inject
    public HandCodeServiceImpl(NotificationService notifications, GeneratePluginConfiguration configuration) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.handCodes = new ArrayList<>();
    }

    @Override
    public int getPriority() {
        return 60;
    }

    @Override
    public boolean existsHandCodedPeer(Path generated) {
        return this.handCodes.stream().anyMatch(handCode -> handCode.equals(generated));
    }

    @Override
    public void onPluginConfigure(Mojo plugin) {
        Predicate<File> predicate = handCodedPath -> handCodedPath.exists() && handCodedPath.isDirectory();
        Function<File, Collection<Path>> mapping = handCodedPath -> {
            Collection<File> handCodes = FileUtils.listFiles(handCodedPath, null, true);
            Function<Path, Path> relativize = path -> handCodedPath.toPath().relativize(path);

            return handCodes.stream().map(File::toPath).map(relativize).collect(Collectors.toList());
        };

        this.notifications.info("Detecting Handwritten Code.");
        this.configuration.getHandCodedPaths().stream().filter(predicate).map(mapping).forEach(this.handCodes::addAll);
    }
}
