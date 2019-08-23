/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.models;

import java.io.File;
import java.io.IOException;
import java.util.List;

public interface ModelSplitter {
    List<File> split(File model) throws IOException;
}
