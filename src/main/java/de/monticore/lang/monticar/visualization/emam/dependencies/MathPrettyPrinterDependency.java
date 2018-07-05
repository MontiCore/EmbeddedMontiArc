package de.monticore.lang.monticar.visualization.emam.dependencies;

import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.paths.PathsServiceImpl;
import de.monticore.lang.monticar.visualization.emam.url.URLService;

import java.io.IOException;
import java.util.logging.Logger;

public class MathPrettyPrinterDependency extends AbstractDependenciesContribution {
    @Inject
    public MathPrettyPrinterDependency(Logger logger, PathsServiceImpl pathsService, URLService urlService) {
        super(logger, pathsService, urlService);
    }

    @Override
    public boolean isDownloaded() {
        return false;
    }

    @Override
    public void download() {

    }

    @Override
    public boolean isInstalled() {
        return false;
    }

    @Override
    public void install() {

    }

    @Override
    public boolean isCleaned() {
        return false;
    }

    @Override
    public void clean() throws IOException {

    }
}
