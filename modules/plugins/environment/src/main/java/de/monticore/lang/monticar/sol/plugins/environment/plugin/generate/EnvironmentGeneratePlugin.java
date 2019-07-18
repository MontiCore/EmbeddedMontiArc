/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generate;

import com.google.inject.Module;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.AbstractGeneratePlugin;
import de.monticore.lang.monticar.sol.plugins.environment.EnvironmentGenerateModule;
import org.apache.maven.plugins.annotations.LifecyclePhase;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@Mojo(name = "generate", inheritByDefault = false, defaultPhase = LifecyclePhase.GENERATE_SOURCES)
public class EnvironmentGeneratePlugin extends AbstractGeneratePlugin {
    @Override
    public Module getModule() {
        return new EnvironmentGenerateModule(this);
    }

    @Parameter(required = true, defaultValue = "node_modules")
    protected File rootDirectory;

    /**
     * @return The directory from which the models should be searched from.
     */
    public File getRootDirectory() {
        return this.rootDirectory;
    }

    @Parameter(required = true)
    protected String baseImage;

    /**
     * @return The image used as base for the resulting Dockerfile.
     */
    public String getBaseImage() {
       return this.baseImage;
    }

    @Parameter(required = true, defaultValue = ".")
    protected File outputPath;

    @Override
    public File getOutputPath() {
        return this.outputPath;
    }

    @Override
    public List<File> getHandCodedPaths() {
        return new ArrayList<>();
    }
}
