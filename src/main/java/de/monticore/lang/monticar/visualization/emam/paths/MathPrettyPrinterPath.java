package de.monticore.lang.monticar.visualization.emam.paths;

import java.nio.file.Path;

public class MathPrettyPrinterPath implements PathsContribution {
    @Override
    public void addToRegistry(PathsService registry) {
        Path projectDirectory = registry.getPath("visualization-emam");
        Path mathPrettyPrinterPath = projectDirectory.resolve("math-pretty-printer");
        Path mathPrettyPrinterZIP = projectDirectory.resolve("math-pretty-printer.zip");
        Path mathPrettyPrinterJAR = mathPrettyPrinterPath.resolve("math-pretty-printer.jar");

        registry.addPath("math-pretty-printer", mathPrettyPrinterPath);
        registry.addPath("math-pretty-printer.zip", mathPrettyPrinterZIP);
        registry.addPath("math-pretty-printer.jar", mathPrettyPrinterJAR);
    }
}
