/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.dependencies;

import java.io.IOException;

public interface DependenciesContribution {
    boolean isInstalled() throws Exception;
    void install() throws IOException;
}
