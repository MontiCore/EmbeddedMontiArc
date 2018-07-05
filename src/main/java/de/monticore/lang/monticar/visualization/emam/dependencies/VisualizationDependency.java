package de.monticore.lang.monticar.visualization.emam.dependencies;

import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.paths.PathsService;
import de.monticore.lang.monticar.visualization.emam.url.URLService;
import net.lingala.zip4j.core.ZipFile;
import net.lingala.zip4j.exception.ZipException;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.nio.file.Path;
import java.util.logging.Logger;

public class VisualizationDependency extends AbstractDependenciesContribution {
    @Inject
    public VisualizationDependency(Logger logger, PathsService pathsService, URLService urlService) {
        super(logger, pathsService, urlService);
    }

    @Override
    public boolean isDownloaded() {
        File visualizationZIP = this.pathsService.getPathAsFile("visualization.zip");

        return this.isInstalled() || visualizationZIP.exists();
    }

    @Override
    public void download() throws IOException {
        URL visualizationURL = this.urlService.getURL("visualization.zip");
        Path visualizationZIP = this.pathsService.getPath("visualization.zip");

        this.logger.info("Downloading \"visualization.zip\"...");
        FileUtils.copyURLToFile(visualizationURL, visualizationZIP.toFile());
    }

    @Override
    public boolean isInstalled() {
        File visualizationJAR = this.pathsService.getPathAsFile("visualization.jar");

        return visualizationJAR.exists();
    }

    @Override
    public void install() throws ZipException {
        File visualizationZIP = this.pathsService.getPathAsFile("visualization.zip");
        Path projectDirectory = this.pathsService.getPath("visualization-emam");
        ZipFile zipFile = new ZipFile(visualizationZIP);

        this.logger.info("Installing \"visualization.zip\"...");
        zipFile.extractAll(projectDirectory.toString());
    }

    @Override
    public boolean isCleaned() {
        File visualizationZIP = this.pathsService.getPathAsFile("visualization.zip");

        return !visualizationZIP.exists();
    }

    @Override
    public void clean() throws IOException {
        File visualizationZIP = this.pathsService.getPathAsFile("visualization.zip");

        this.logger.info("Deleting \"visualization.zip\"...");
        FileUtils.forceDelete(visualizationZIP);
    }
}
