/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.artifact.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorSetup;
import de.monticore.io.paths.IterablePath;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.configuration.ArtifactConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorSetupContribution;

@Singleton
public class ArtifactGeneratorSetup implements GeneratorSetupContribution {
    protected final ArtifactConfiguration configuration;

    @Inject
    protected ArtifactGeneratorSetup(ArtifactConfiguration configuration) {
        this.configuration = configuration;
    }

    @Override
    public void setup(GeneratorSetup setup) {
        setup.setTracing(false);
        setup.setOutputDirectory(this.configuration.getSourceCodeOutputPath());
        setup.setHandcodedPath(IterablePath.from(this.configuration.getHandCodedPaths(), "ts"));
    }
}
