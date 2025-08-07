/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.artifact.plugin.configuration;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;

import java.util.List;

public interface ArtifactConfiguration extends GeneratePluginConfiguration {
    List<String> getRootModels();
}
