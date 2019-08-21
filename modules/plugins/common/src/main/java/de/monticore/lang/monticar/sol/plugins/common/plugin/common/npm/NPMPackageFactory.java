/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import java.io.File;

/**
 * Interface of a factory which creates NPMPackage instances.
 */
public interface NPMPackageFactory {
    /**
     * Creates a NPMPackage instance for a given path.
     * @param path The path of the 'package.json' to which the NPMPackage instance should be pointing to.
     * @return The created NPMPackage instance.
     */
    NPMPackage create(File path);
}
