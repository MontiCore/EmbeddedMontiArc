/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration;

import org.apache.maven.project.MavenProject;

import java.io.File;

public interface PluginConfiguration {
    /**
     * @return The MavenProject object representing the Maven Project in which the plugin is executed.
     */
    MavenProject getMavenProject();

    /**
     * @return The path in which files holding the state of the generator should be saved.
     */
    File getStatePath();
}
