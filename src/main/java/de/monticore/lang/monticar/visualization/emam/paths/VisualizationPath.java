package de.monticore.lang.monticar.visualization.emam.paths;

import java.nio.file.Path;

public class VisualizationPath implements PathsContribution {
    @Override
    public void addToRegistry(PathsService registry) {
        Path projectDirectory = registry.getPath("visualization-emam");
        Path visualizationPath = projectDirectory.resolve("visualisation");
        Path visualizationZIP = projectDirectory.resolve("visualisation.zip");
        Path visualizationJAR = visualizationPath.resolve("embeddedmontiarc-svggenerator.jar");

        registry.addPath("visualization", visualizationPath);
        registry.addPath("visualization.jar", visualizationJAR);
        registry.addPath("visualization.zip", visualizationZIP);
    }
}
