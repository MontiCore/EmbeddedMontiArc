/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.paths;

import java.nio.file.Path;

public class MathPrettyPrinterPath implements PathsContribution {
    @Override
    public void addToRegistry(PathsService registry) {
        Path projectDirectory = registry.getPath("visualization-emam");
        Path mathPrettyPrinterJAR = projectDirectory.resolve("math-pretty-printer.jar");

        registry.addPath("math-pretty-printer.jar", mathPrettyPrinterJAR);
    }
}
