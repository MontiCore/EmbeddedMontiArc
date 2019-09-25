/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import java.io.File;
import java.util.List;
import java.util.Optional;

/**
 * Interface of a resolver which resolves NodeModules.
 */
public interface NodeModulesResolver {
    /**
     * Resolves all NodeModules from a given start path.
     * @param start The path from which the resolving process should start.
     * @return A list of resolved NodeModules.
     */
    List<NodeModules> resolve(File start);

    /**
     * Resolves the first NodeModules encountered from a given start path.
     * @param start The path from which the resolving process should start.
     * @return The first resolved NodeModules if present, empty otherwise.
     */
    Optional<NodeModules> resolveFirst(File start);
}
