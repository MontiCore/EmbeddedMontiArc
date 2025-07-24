/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.hc;

import java.nio.file.Path;
import java.util.Set;

public interface HandCodeRegistry {
    /**
     * @return The handwritten files as paths.
     */
    Set<Path> getHandCodes();
}
