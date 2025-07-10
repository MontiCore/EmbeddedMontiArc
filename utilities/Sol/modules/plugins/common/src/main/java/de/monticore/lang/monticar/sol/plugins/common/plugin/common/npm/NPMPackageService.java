/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;

/**
 * A service which can be used to perform various operations on NPMPackages.
 */
public interface NPMPackageService {
    /**
     * Returns the package located in the operation context of the plugin.
     * @return The package located in the operation context of the plugin.
     */
    Optional<SolPackage> getCurrentPackage();

    /**
     * Returns all NPMPackages found during resolution.
     * @return All NPMPackages found during resolution.
     */
    List<NPMPackage> getAllPackages();

    /**
     * Returns the NPMPackage with the given name if present.
     * @param name The name used for the resolution request.
     * @return The resolved NPMPackage if present, empty otherwise.
     */
    Optional<NPMPackage> resolve(String name); // TODO: Handle version together with name.

    /**
     * Returns the NPMPackage matching the given predicate.
     * @param predicate The predicate used as filter for the resolution.
     * @return The resolved NPMPackage if present, empty otherwise.
     */
    List<NPMPackage> resolveMany(Predicate<NPMPackage> predicate);
}
