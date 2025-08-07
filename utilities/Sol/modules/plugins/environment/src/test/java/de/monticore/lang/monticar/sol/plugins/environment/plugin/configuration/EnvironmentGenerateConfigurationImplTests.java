/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration;

import de.monticore.lang.monticar.sol.plugins.environment.plugin.EnvironmentGeneratePlugin;
import org.apache.maven.project.MavenProject;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class EnvironmentGenerateConfigurationImplTests {
    @Mock EnvironmentGeneratePlugin plugin;

    EnvironmentGenerateConfigurationImpl configuration;
    File directory = new File("/abc/def");

    @BeforeEach
    void before() {
        MavenProject mavenProject = mock(MavenProject.class);

        when(plugin.getMavenProject()).thenReturn(mavenProject);
        when(plugin.getOutputPath()).thenReturn(directory);
        when(plugin.getRootModel()).thenReturn("de.monticore.lang.monticar.sol.tests.environment.Environment");

        configuration = new EnvironmentGenerateConfigurationImpl(plugin);
    }

    @Test
    void testGetHandCodedPaths() {
        assertEquals(0, configuration.getHandCodedPaths().size(), "There should not be any handwritten code paths.");
    }

    @Test
    void testGetSourceCodeOutputPath() {
        assertEquals(directory, configuration.getSourceCodeOutputPath(), "Directories do not match.");
    }

    @Test
    void testGetDefaultHandCodedPath() {
        assertNull(configuration.getDefaultHandCodedPath(), "There should not be any handwritten code paths.");
    }

    @Test
    void testGetRootModel() {
        assertEquals("de.monticore.lang.monticar.sol.tests.environment.Environment", configuration.getRootModel(), "Root Models do not match.");
    }
}
