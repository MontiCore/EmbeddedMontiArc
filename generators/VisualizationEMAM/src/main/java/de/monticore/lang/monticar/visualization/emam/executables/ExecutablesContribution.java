/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.executables;

import java.io.IOException;

public interface ExecutablesContribution {
    void prepare();
    void execute() throws IOException;
}
