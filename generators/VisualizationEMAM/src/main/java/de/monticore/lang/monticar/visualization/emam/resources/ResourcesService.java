/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.resources;

import java.io.IOException;
import java.io.InputStream;

public interface ResourcesService {
    boolean isPackaged();
    void addResource(String id, String resource);
    String getResource(String id);
    InputStream getResourceAsStream(String id) throws IOException;
}
