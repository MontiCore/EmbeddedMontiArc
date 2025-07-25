/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin;

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

    @Parameter(required = true)
    protected String rootModel;

    public String getRootModel() {
        return this.rootModel;
    }

    @Parameter(required = true, defaultValue = ".")
    protected File outputPath;

    /**
     * @return The path where the Dockerfile should be generated to.
     */
    @Override
    public File getOutputPath() {
        return this.outputPath;
    }

    @Override
    public List<File> getHandCodedPaths() {
        return new ArrayList<>();
    }
}
