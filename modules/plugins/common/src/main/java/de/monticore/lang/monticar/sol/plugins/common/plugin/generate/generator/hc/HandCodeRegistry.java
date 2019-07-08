/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
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
