/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.resources;

public class VisualizationResource implements ResourcesContribution {
    @Override
    public void addToRegistry(ResourcesService registry) {
        registry.addResource("visualization.jar", "visualisation/embeddedmontiarc-svggenerator.jar");
    }
}
