/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.models;

import java.io.IOException;
import java.nio.file.Path;

public interface ModelPathVisitor {
    void visit(Path modelPath, ModelsService registry) throws IOException;
}
