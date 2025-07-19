/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.artifact.plugin.validator;

import org.apache.maven.plugin.MojoExecutionException;

public interface ArtifactValidator {
    void validate() throws MojoExecutionException;
}
