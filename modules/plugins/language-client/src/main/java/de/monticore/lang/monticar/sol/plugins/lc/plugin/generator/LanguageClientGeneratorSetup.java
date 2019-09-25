/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorSetup;
import de.monticore.io.paths.IterablePath;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorSetupContribution;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;

@Singleton
public class LanguageClientGeneratorSetup implements GeneratorSetupContribution {
    protected final LanguageClientConfiguration configuration;

    @Inject
    protected LanguageClientGeneratorSetup(LanguageClientConfiguration configuration) {
        this.configuration = configuration;
    }

    @Override
    public void setup(GeneratorSetup setup) {
        setup.setTracing(false);
        setup.setOutputDirectory(this.configuration.getSourceCodeOutputPath());
        setup.setHandcodedPath(IterablePath.from(this.configuration.getHandCodedPaths(), "ts"));
    }
}
