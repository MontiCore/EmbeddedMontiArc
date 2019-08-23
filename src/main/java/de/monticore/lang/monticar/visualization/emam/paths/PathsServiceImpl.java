/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.paths;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.visualization.emam.application.Application;
import de.monticore.lang.monticar.visualization.emam.application.ApplicationContribution;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;

@Singleton
public class PathsServiceImpl implements PathsService, ApplicationContribution {
    protected final Logger logger;
    protected final Set<PathsContribution> contributions;
    protected final Map<String, Path> paths;

    @Inject
    public PathsServiceImpl(Logger logger, Set<PathsContribution> contributions) {
        this.logger = logger;
        this.contributions = contributions;
        this.paths = new HashMap<>();

        this.addUserHome();
        this.addEmbeddedMontiArcDirectory();
        this.addProjectDirectory();
    }

    protected void addUserHome() {
        String pathString = System.getProperty("user.home");
        Path path = Paths.get(pathString);

        this.paths.put("~", path);
    }

    protected void addEmbeddedMontiArcDirectory() {
        Path userhome = this.paths.get("~");
        Path embeddedMontiArcDirectory = userhome.resolve(".embeddedmontiarc");

        this.paths.put(".embeddedmontiarc", embeddedMontiArcDirectory);
    }

    protected void addProjectDirectory() {
        Path embeddedMontiArcDirectory = this.paths.get(".embeddedmontiarc");
        Path projectDirectory = embeddedMontiArcDirectory.resolve("visualization-emam");

        this.paths.put("visualization-emam", projectDirectory);
    }

    @Override
    public void addPath(String id, Path path) {
        if (this.paths.containsKey(id)) this.logger.warning("A path under the id of " + id + " was already registered");
        else this.paths.put(id, path);
    }

    @Override
    public Path getPath(String id) {
        return this.paths.get(id);
    }

    @Override
    public File getPathAsFile(String id) {
        return this.getPath(id).toFile();
    }

    @Override
    public void configure(Application application) {
        this.logger.info("Adding Paths to PathsService...");

        for (PathsContribution contribution : this.contributions) {
            contribution.addToRegistry(this);
        }
    }
}
