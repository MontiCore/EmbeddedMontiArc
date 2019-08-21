/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import org.json.JSONArray;

/**
 * An interface representing a Theia package.
 */
public interface TheiaPackage extends NPMPackage {
    /**
     * Returns all entries of the 'theiaExtensions' section of the package.
     * @return All entries of the 'theiaExtensions' section of the package.
     */
    JSONArray getExtensions();
}
