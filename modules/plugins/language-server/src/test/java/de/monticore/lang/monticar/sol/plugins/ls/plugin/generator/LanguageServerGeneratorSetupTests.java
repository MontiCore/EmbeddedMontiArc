/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.generator;

import de.monticore.generating.GeneratorSetup;
import de.monticore.io.paths.IterablePath;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.generator.LanguageServerGeneratorSetup;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
public class LanguageServerGeneratorSetupTests {
    @Mock LanguageServerConfiguration configuration;

    @InjectMocks LanguageServerGeneratorSetup genSetup;

    @Test
    void testSetup() {
        File outputPath = new File("some/output/directory");
        List<File> handCodedPaths = new ArrayList<>();
        File handCodePath = new File("some/hand/code/directory/some/File.java");
        GeneratorSetup setup = mock(GeneratorSetup.class);

        handCodedPaths.add(handCodePath);

        when(configuration.getSourceCodeOutputPath()).thenReturn(outputPath);
        when(configuration.getHandCodedPaths()).thenReturn(handCodedPaths);

        genSetup.setup(setup);

        verify(setup).setTracing(false);
        verify(setup).setOutputDirectory(outputPath);
        verify(setup).setHandcodedPath(any(IterablePath.class));
    }
}
