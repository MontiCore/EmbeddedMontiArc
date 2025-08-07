/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import org.json.JSONArray;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

/**
 * An interface representing a Sol package.
 */
public interface SolPackage extends NPMPackage {
    /**
     * Returns an entry in the direction section of this sol package as string.
     * @param identifier The identifier of the entry.
     * @return An entry in the direction section of this sol package as string.
     */
    Optional<String> getDirectory(String identifier);

    /**
     * Returns an entry in the direction section of this sol package as path.
     * @param identifier The identifier of the entry.
     * @return An entry in the direction section of this sol package as path.
     */
    Optional<Path> getDirectoryAsPath(String identifier);

    /**
     * Returns all direct dependencies of this package which are SolPackages.
     * @return All direct dependencies of this package which are SolPackages.
     */
    List<SolPackage> getSolDependencies();

    /**
     * Returns all dependencies of this package which are SolPackages, transitive dependencies included.
     * @return All dependencies of this package which are SolPackages, transitive dependencies included.
     */
    List<SolPackage> getAllSolDependencies();

    Optional<JSONArray> getExtensions();
}
