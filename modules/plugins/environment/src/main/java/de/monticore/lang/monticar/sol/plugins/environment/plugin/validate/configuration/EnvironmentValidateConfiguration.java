/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.configuration;

import java.io.File;

public interface EnvironmentValidateConfiguration {
    /**
     * @return The directory from which the models should be searched from.
     */
    File getRootDirectory();
}
