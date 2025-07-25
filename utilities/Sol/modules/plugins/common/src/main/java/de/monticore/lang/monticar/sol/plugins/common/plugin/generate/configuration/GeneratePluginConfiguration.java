/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;

import java.io.File;
import java.util.List;

public interface GeneratePluginConfiguration extends PluginConfiguration {
    /**
     * @return The path used as root of the output.
     */
    File getOutputPath();

    /**
     * @return The path used as root of the source code output (without the package structure).
     */
    File getSourceCodeOutputPath();

    /**
     * @return A list of paths where the handwritten code is located.
     */
    List<File> getHandCodedPaths();
}
