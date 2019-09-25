/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import java.io.File;
import java.util.List;
import java.util.Optional;

/**
 * An interface representing the a NPM package.
 */
public interface NPMPackage {
    /**
     * Returns the location of the 'package.json'.
     * @return The location of the 'package.json'.
     */
    File getPath();

    /**
     * Returns the name of the package.
     * @return The name of the package if present, empty otherwise.
     */
    Optional<String> getName();

    /**
     * Returns a list of NPMPackages representing the dependencies of this package.
     * @return A list of NPMPackages representing the dependencies of this package.
     */
    List<NPMPackage> getDependencies();

    /**
     * Checks whether an attribute exists for a given key.
     * @param key The given for which existence should be checked.
     * @return True if an attribute for the given key exists, false otherwise.
     */
    boolean hasAttribute(String key);

    /**
     * Returns the attribute for the given key.
     * @param key The key of the attribute.
     * @param <T> The attribute to be fetched casted to T.
     * @return The fetched attribute casted to T.
     */
    <T> T getAttribute(String key);

    /**
     * Checks whether this package is a Theia package.
     * @return True if this package is a Theia package, false otherwise.
     */
    boolean isTheiaPackage();

    /**
     * Checks whether this package is a Sol package.
     * @return True if this package is a Sol package, false otherwise.
     */
    boolean isSolPackage();

    /**
     * Returns this package as Sol package.
     * @return The package as Sol package.
     */
    Optional<SolPackage> getAsSolPackage();

    /**
     * Returns this package as Theia package.
     * @return The package as Theia package.
     */
    Optional<TheiaPackage> getAsTheiaPackage();
}
