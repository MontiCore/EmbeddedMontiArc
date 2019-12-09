/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.path;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import org.apache.maven.project.MavenProject;

import java.nio.file.Path;
import java.util.EnumMap;
import java.util.Map;

@Singleton
public class PathResolverImpl implements PathResolver {
    protected final NotificationService notifications;
    protected final PluginConfiguration configuration;
    protected final Map<CommonLiterals, Path> cache;

    @Inject
    protected PathResolverImpl(NotificationService notifications, PluginConfiguration configuration) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.cache = new EnumMap<>(CommonLiterals.class);
    }

    @Override
    public Path resolve(CommonLiterals origin, String path) {
        return this.resolveOrigin(origin).resolve(path);
    }

    @Override
    public Path resolve(CommonLiterals origin, Path path) {
        return this.resolveOrigin(origin).resolve(path);
    }

    protected Path resolveOrigin(CommonLiterals origin) {
        if (this.cache.containsKey(origin)) return this.cache.get(origin);
        else return this.doResolveOrigin(origin);
    }

    protected Path doResolveOrigin(CommonLiterals origin) {
        switch (origin) {
            case PARENT: return this.resolveParentProjectDirectory();
            case ROOT: return this.resolveRootProjectDirectory();
            default: return this.resolveCurrentWorkingDirectory();
        }
    }

    protected Path resolveCurrentWorkingDirectory() {
        Path directory = this.configuration.getMavenProject().getBasedir().toPath();

        this.cache.put(CommonLiterals.CWD, directory);
        return directory;
    }

    protected Path resolveParentProjectDirectory() {
        MavenProject parentProject = this.configuration.getMavenProject().getParent();
        Path directory = parentProject.getBasedir().toPath().toAbsolutePath();

        this.cache.put(CommonLiterals.PARENT, directory);
        return directory;
    }

    protected Path resolveRootProjectDirectory() {
        MavenProject rootProject = this.resolveRootProject(this.configuration.getMavenProject());
        Path directory = rootProject.getBasedir().toPath().toAbsolutePath();

        this.cache.put(CommonLiterals.ROOT, directory);
        return directory;
    }

    protected MavenProject resolveRootProject(MavenProject current) {
        if (current.getParent() == null) return current;
        else return this.resolveRootProject(current.getParent());
    }
}
