/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.dependencies;

import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.paths.PathsService;
import de.monticore.lang.monticar.visualization.emam.resources.ResourcesService;

import java.io.IOException;
import java.util.logging.Logger;

public class VisualizationDependency extends AbstractDependenciesContribution {
    @Inject
    public VisualizationDependency(Logger logger, PathsService pathsService, ResourcesService resourcesService) {
        super(logger, pathsService, resourcesService);
    }

    @Override
    public boolean isInstalled() throws Exception {
        return super.isInstalled("visualization.jar");
    }

    @Override
    public void install() throws IOException {
        super.install("visualization.jar");
    }
}
