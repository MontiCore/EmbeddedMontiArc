/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.validate;

import com.google.inject.Module;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.AbstractPlugin;
import de.monticore.lang.monticar.sol.plugins.environment.EnvironmentValidateModule;
import org.apache.maven.plugins.annotations.LifecyclePhase;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;

@Mojo(name = "validate", inheritByDefault = false, defaultPhase = LifecyclePhase.VALIDATE)
public class EnvironmentValidatePlugin extends AbstractPlugin {
    @Override
    public Module getModule() {
        return new EnvironmentValidateModule(this);
    }

    @Parameter(required = true, defaultValue = "models/environment")
    protected File rootDirectory;

    /**
     * @return The directory from which the models should be searched from.
     */
    public File getRootDirectory() {
        return this.rootDirectory;
    }
}
