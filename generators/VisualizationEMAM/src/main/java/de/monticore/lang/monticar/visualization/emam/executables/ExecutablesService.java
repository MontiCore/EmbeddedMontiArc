/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.executables;

import java.io.IOException;

public interface ExecutablesService {
    void prepare();
    void execute() throws IOException;
}
