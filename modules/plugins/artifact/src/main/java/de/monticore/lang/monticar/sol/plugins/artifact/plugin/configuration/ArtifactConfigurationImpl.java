/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.artifact.plugin.configuration;

import com.google.common.base.Preconditions;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.ArtifactPlugin;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.AbstractGeneratePluginConfiguration;

import java.io.File;
import java.util.List;

@Singleton
public class ArtifactConfigurationImpl extends AbstractGeneratePluginConfiguration implements ArtifactConfiguration {
    private final ArtifactPlugin plugin;

    @Inject
    protected ArtifactConfigurationImpl(ArtifactPlugin plugin) {
        super(plugin);

        this.plugin = plugin;
    }

    @Override
    protected File getDefaultHandCodedPath() {
        return this.resolveFromBaseDirectory("src");
    }

    @Override
    public List<String> getRootModels() {
        Preconditions.checkNotNull(this.plugin);

        return this.plugin.getRootModels();
    }

    @Override
    public File getSourceCodeOutputPath() {
        return new File(this.getOutputPath(), "src-gen");
    }
}
