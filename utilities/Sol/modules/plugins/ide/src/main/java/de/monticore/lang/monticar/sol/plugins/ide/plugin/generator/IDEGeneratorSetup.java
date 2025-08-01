/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorSetup;
import de.monticore.io.paths.IterablePath;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorSetupContribution;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.configuration.IDEConfiguration;

@Singleton
public class IDEGeneratorSetup implements GeneratorSetupContribution {
    protected final IDEConfiguration configuration;

    @Inject
    protected IDEGeneratorSetup(IDEConfiguration configuration) {
        this.configuration = configuration;
    }

    @Override
    public void setup(GeneratorSetup setup) {
        setup.setTracing(false);
        setup.setOutputDirectory(this.configuration.getSourceCodeOutputPath());
        setup.setHandcodedPath(IterablePath.from(this.configuration.getHandCodedPaths(), "ts"));
    }
}
