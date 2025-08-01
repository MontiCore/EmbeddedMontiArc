/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin;

import com.google.common.collect.ImmutableList;
import com.google.inject.Module;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.AbstractGeneratePlugin;
import de.monticore.lang.monticar.sol.plugins.ide.IDEGenerateModule;
import org.apache.maven.plugins.annotations.LifecyclePhase;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.util.List;

import static com.google.common.base.MoreObjects.firstNonNull;

@Mojo(name = "generate", inheritByDefault = false, defaultPhase = LifecyclePhase.GENERATE_SOURCES)
public class IDEPlugin extends AbstractGeneratePlugin {
    @Parameter(required = true)
    protected String rootModel;

    public String getRootModel() {
        return this.rootModel;
    }

    @Parameter(defaultValue = ".")
    protected File outputDirectory;

    /**
     * @return The output directory. (Default: .).
     */
    public File getOutputPath() {
        return this.outputDirectory;
    }

    @Parameter
    protected List<File> handCodedPaths;

    @Override
    public List<File> getHandCodedPaths() {
        return firstNonNull(this.handCodedPaths, ImmutableList.of());
    }

    @Override
    public Module getModule() {
        return new IDEGenerateModule(this);
    }
}
