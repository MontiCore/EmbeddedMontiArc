/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.paths;

import java.nio.file.Path;

public class VisualizationPath implements PathsContribution {
    @Override
    public void addToRegistry(PathsService registry) {
        Path projectDirectory = registry.getPath("visualization-emam");
        Path visualizationJAR = projectDirectory.resolve("embeddedmontiarc-svggenerator.jar");

        registry.addPath("visualization.jar", visualizationJAR);
    }
}
