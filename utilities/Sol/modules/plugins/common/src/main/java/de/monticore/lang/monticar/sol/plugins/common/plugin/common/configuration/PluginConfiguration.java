/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration;

import org.apache.maven.project.MavenProject;

public interface PluginConfiguration {
    /**
     * @return The MavenProject object representing the Maven Project in which the plugin is executed.
     */
    MavenProject getMavenProject();
}
