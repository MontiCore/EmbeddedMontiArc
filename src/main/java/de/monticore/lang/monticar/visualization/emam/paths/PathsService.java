/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.paths;

import java.io.File;
import java.nio.file.Path;

public interface PathsService {
    void addPath(String id, Path path);
    Path getPath(String id);
    File getPathAsFile(String id);
}
