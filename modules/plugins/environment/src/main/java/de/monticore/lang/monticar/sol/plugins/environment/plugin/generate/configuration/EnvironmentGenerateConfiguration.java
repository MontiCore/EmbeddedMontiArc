/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.configuration;

import de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.configuration.EnvironmentValidateConfiguration;

import java.io.File;

public interface EnvironmentGenerateConfiguration extends EnvironmentValidateConfiguration {
    /**
     * @return The image used as base for the resulting Dockerfile.
     */
    String getBaseImage();

    /**
     * @return The output directory for the generated Dockerfile as File.
     */
    File getOutputPath();
}
