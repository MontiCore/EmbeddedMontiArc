/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.configuration;

import com.google.common.base.Preconditions;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.AbstractGeneratePluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.IDEPlugin;

import java.io.File;

@Singleton
public class IDEConfigurationImpl extends AbstractGeneratePluginConfiguration implements IDEConfiguration {
    private final IDEPlugin plugin;

    @Inject
    protected IDEConfigurationImpl(IDEPlugin plugin) {
        super(plugin);

        this.plugin = plugin;
    }

    @Override
    protected File getDefaultHandCodedPath() {
        return this.resolveFromBaseDirectory("src");
    }

    @Override
    public String getRootModel() {
        Preconditions.checkNotNull(this.plugin);

        return this.plugin.getRootModel();
    }

    @Override
    public File getSourceCodeOutputPath() {
        return new File(this.getOutputPath(), "src-gen");
    }
}
