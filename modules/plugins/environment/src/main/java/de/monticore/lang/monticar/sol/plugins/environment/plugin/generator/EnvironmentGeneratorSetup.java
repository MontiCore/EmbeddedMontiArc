/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorSetup;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorSetupContribution;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration.EnvironmentGenerateConfiguration;

@Singleton
public class EnvironmentGeneratorSetup implements GeneratorSetupContribution {
    protected final EnvironmentGenerateConfiguration configuration;

    @Inject
    protected EnvironmentGeneratorSetup(EnvironmentGenerateConfiguration configuration) {
        this.configuration = configuration;
    }

    @Override
    public void setup(GeneratorSetup setup) {
        setup.setTracing(false);
        setup.setOutputDirectory(this.configuration.getOutputPath());
    }
}
