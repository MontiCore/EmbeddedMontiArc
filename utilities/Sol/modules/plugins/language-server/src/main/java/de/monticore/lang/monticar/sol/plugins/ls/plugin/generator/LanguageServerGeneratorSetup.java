/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorSetup;
import de.monticore.io.paths.IterablePath;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorSetupContribution;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration;

@Singleton
public class LanguageServerGeneratorSetup implements GeneratorSetupContribution {
    protected final LanguageServerConfiguration configuration;

    @Inject
    public LanguageServerGeneratorSetup(LanguageServerConfiguration configuration) {
        this.configuration = configuration;
    }

    @Override
    public void setup(GeneratorSetup setup) {
        setup.setTracing(false);
        setup.setOutputDirectory(this.configuration.getSourceCodeOutputPath());
        setup.setHandcodedPath(IterablePath.from(this.configuration.getHandCodedPaths(), "java"));
    }
}
