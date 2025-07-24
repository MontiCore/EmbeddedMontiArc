/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import java.io.File;
import java.util.List;

/**
 * An interface representing a 'node_modules' folder in the system.
 */
public interface NodeModules {
    /**
     * Returns the location where the 'node_modules' folder is located.
     * @return The location where the 'node_modules' folder is located.
     */
    File getPath();

    /**
     * Returns the NPMPackages located in this 'node_modules' folder.
     * @return The NPMPackages located in this 'node_modules' folder.
     */
    List<NPMPackage> getPackages();
}
