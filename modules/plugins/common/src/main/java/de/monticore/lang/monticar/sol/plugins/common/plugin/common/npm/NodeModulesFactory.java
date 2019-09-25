/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import java.io.File;

/**
 * Interface of a factory which creates NodeModules instances.
 */
public interface NodeModulesFactory {
    /**
     * Creates a NodeModules instance for a given path.
     * @param path The path to which the NodeModule instance should be pointing to.
     * @return The created NodeModules instance.
     */
    NodeModules create(File path);
}
