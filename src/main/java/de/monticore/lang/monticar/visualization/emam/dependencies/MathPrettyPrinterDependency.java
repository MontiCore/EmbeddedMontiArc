/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.dependencies;

import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.paths.PathsServiceImpl;
import de.monticore.lang.monticar.visualization.emam.resources.ResourcesService;

import java.io.IOException;
import java.util.logging.Logger;

public class MathPrettyPrinterDependency extends AbstractDependenciesContribution {
    @Inject
    public MathPrettyPrinterDependency(Logger logger, PathsServiceImpl pathsService, ResourcesService resourcesService) {
        super(logger, pathsService, resourcesService);
    }

    @Override
    public boolean isInstalled() throws Exception {
        return super.isInstalled("math-pretty-printer.jar");
    }

    @Override
    public void install() throws IOException {
        super.install("math-pretty-printer.jar");
    }
}
