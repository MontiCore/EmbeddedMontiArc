/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.validator;

import org.apache.maven.plugin.MojoExecutionException;

public interface IDEValidator {
    void validate() throws MojoExecutionException;
}
