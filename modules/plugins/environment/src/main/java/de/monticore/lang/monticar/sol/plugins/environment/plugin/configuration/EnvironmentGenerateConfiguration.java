/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;

import java.io.File;

public interface EnvironmentGenerateConfiguration extends GeneratePluginConfiguration {
    /**
     * @return The qualified name of the root model.
     */
    String getRootModel();

    /**
     * @return The output directory for the generated Dockerfile as File.
     */
    File getOutputPath();
}
