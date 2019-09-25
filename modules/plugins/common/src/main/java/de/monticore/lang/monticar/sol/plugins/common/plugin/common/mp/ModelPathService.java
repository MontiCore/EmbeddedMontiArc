/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;

/**
 * A service which can be used to query the ModelPath.
 */
public interface ModelPathService {
    /**
     * A method which can be used to resolve the ModelPath of a given SolPackage.
     * @param root The SolPackage from which the ModelPath should be resolved.
     * @return The resolved ModelPath.
     */
    ModelPath resolve(SolPackage root);
}
