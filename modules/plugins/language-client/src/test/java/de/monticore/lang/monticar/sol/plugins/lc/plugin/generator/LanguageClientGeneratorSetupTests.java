/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator;

import de.monticore.generating.GeneratorSetup;
import de.monticore.io.paths.IterablePath;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.io.File;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
public class LanguageClientGeneratorSetupTests {
    @Mock LanguageClientConfiguration configuration;

    @InjectMocks LanguageClientGeneratorSetup genSetup;

    @Test
    void testSetup() {
        GeneratorSetup setup = mock(GeneratorSetup.class);
        File outputPath = new File("sourcecode");
        List<File> handCodePaths = new ArrayList<>();

        handCodePaths.add(Paths.get(".").toFile());

        when(configuration.getSourceCodeOutputPath()).thenReturn(outputPath);
        when(configuration.getHandCodedPaths()).thenReturn(handCodePaths);

        genSetup.setup(setup);

        verify(setup).setTracing(false);
        verify(setup).setOutputDirectory(outputPath);
        verify(setup).setHandcodedPath(any(IterablePath.class));
    }
}
