package de.monticore.lang.monticar.visualization.emam.dependencies;

import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.paths.PathsServiceImpl;
import de.monticore.lang.monticar.visualization.emam.url.URLService;
import net.lingala.zip4j.core.ZipFile;
import net.lingala.zip4j.exception.ZipException;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.nio.file.Path;
import java.util.logging.Logger;

public class MathPrettyPrinterDependency extends AbstractDependenciesContribution {
    @Inject
    public MathPrettyPrinterDependency(Logger logger, PathsServiceImpl pathsService, URLService urlService) {
        super(logger, pathsService, urlService);
    }

    @Override
    public boolean isDownloaded() {
        File mppZIP = this.pathsService.getPathAsFile("math-pretty-printer.zip");

        return this.isInstalled() || mppZIP.exists();
    }

    @Override
    public void download() throws IOException {
        URL mppURL = this.urlService.getURL("math-pretty-printer.zip");
        Path mppZIP = this.pathsService.getPath("math-pretty-printer.zip");

        this.logger.info("Downloading \"math-pretty-printer.zip\"...");
        FileUtils.copyURLToFile(mppURL, mppZIP.toFile());
    }

    @Override
    public boolean isInstalled() {
        File mppJAR = this.pathsService.getPathAsFile("math-pretty-printer.jar");

        return mppJAR.exists();
    }

    @Override
    public void install() throws ZipException {
        File mppZIP = this.pathsService.getPathAsFile("math-pretty-printer.zip");
        Path projectDirectory = this.pathsService.getPath("visualization-emam");
        ZipFile zipFile = new ZipFile(mppZIP);

        this.logger.info("Installing \"math-pretty-printer.zip\"...");
        zipFile.extractAll(projectDirectory.toString());
    }

    @Override
    public boolean isCleaned() {
        File mppZIP = this.pathsService.getPathAsFile("math-pretty-printer.zip");

        return !mppZIP.exists();
    }

    @Override
    public void clean() throws IOException {
        File mppZIP = this.pathsService.getPathAsFile("math-pretty-printer.zip");

        this.logger.info("Deleting \"math-pretty-printer.zip\"...");
        FileUtils.forceDelete(mppZIP);
    }
}
