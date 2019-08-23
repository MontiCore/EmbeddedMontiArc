/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.models;

import java.io.File;
import java.util.Collection;
import java.util.List;

public interface ModelsService {
    void addModel(File model);
    void addAllModels(Collection<File> models);
    List<File> getModels();
    void clearModels();
}
