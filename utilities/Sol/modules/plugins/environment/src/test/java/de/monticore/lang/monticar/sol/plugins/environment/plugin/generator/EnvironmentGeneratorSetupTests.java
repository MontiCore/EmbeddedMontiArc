/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator;

import de.monticore.generating.GeneratorSetup;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration.EnvironmentGenerateConfiguration;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;

import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class EnvironmentGeneratorSetupTests {
    @Mock EnvironmentGenerateConfiguration configuration;

    @InjectMocks EnvironmentGeneratorSetup genSetup;

    @Test
    void testSetup() {
        File outputPath = new File(".");

        when(configuration.getOutputPath()).thenReturn(outputPath);

        GeneratorSetup setup = mock(GeneratorSetup.class);

        genSetup.setup(setup);

        verify(setup).setTracing(false);
        verify(setup).setOutputDirectory(outputPath);
    }
}
