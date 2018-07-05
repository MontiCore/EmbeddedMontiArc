package de.monticore.lang.monticar.visualization.emam.dependencies;

import net.lingala.zip4j.exception.ZipException;

import java.io.IOException;

public interface DependenciesContribution {
    boolean isDownloaded();
    void download() throws IOException;

    boolean isInstalled();
    void install() throws ZipException;

    boolean isCleaned();
    void clean() throws IOException;
}
