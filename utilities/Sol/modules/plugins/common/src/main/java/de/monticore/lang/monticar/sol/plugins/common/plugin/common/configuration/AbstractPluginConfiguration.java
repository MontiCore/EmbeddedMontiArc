/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration;

import com.google.common.base.Preconditions;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.AbstractPlugin;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import org.apache.maven.project.MavenProject;

import java.io.File;

public abstract class AbstractPluginConfiguration implements PluginConfiguration, PluginContribution {
    private final AbstractPlugin plugin;

    protected AbstractPluginConfiguration(AbstractPlugin plugin) {
        this.plugin = plugin;
    }

    @Override
    public int getPriority() {
        return 50000;
    }

    @Override
    public MavenProject getMavenProject() {
        Preconditions.checkNotNull(this.plugin);

        return this.plugin.getMavenProject();
    }

    protected File resolveFromBaseDirectory(File file) {
        return this.resolveFromBaseDirectory(file.getPath());
    }

    protected File resolveFromBaseDirectory(String filePath) {
        File file = new File(filePath);

        return file.isAbsolute() ? file : new File(this.getMavenProject().getBasedir(), filePath);
    }
}
