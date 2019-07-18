/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.configuration;

import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.EnvironmentGeneratePlugin;
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

        configuration = new EnvironmentGenerateConfigurationImpl(plugin);
    }

    @Test
    void testGetBaseImage() {
        when(plugin.getBaseImage()).thenReturn("BaseImage:latest");

        assertEquals("BaseImage:latest", configuration.getBaseImage(), "Base Image does not match");
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
    void testGetRootDirectory() {
        when(plugin.getRootDirectory()).thenReturn(directory);

        assertEquals(directory, configuration.getRootDirectory(), "Root Directory does not match.");
    }

    @Test
    void testGetStatePath() {
        File expectedPath = new File(directory, "state");

        assertEquals(expectedPath, configuration.getStatePath(), "State Path does not match.");
    }
}
