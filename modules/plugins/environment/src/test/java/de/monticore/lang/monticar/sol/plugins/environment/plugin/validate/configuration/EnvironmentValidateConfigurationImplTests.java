/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.configuration;

import de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.EnvironmentValidatePlugin;
import org.apache.maven.project.MavenProject;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.io.File;
import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
public class EnvironmentValidateConfigurationImplTests {
    @Mock EnvironmentValidatePlugin plugin;

    @InjectMocks EnvironmentValidateConfigurationImpl configuration;

    @Test
    void testGetStatePath() {
        MavenProject mavenProject = mock(MavenProject.class);
        File baseDir = new File("");
        File statePath = new File(baseDir, "state");

        when(plugin.getMavenProject()).thenReturn(mavenProject);
        when(mavenProject.getBasedir()).thenReturn(baseDir);

        assertEquals(statePath, configuration.getStatePath(), "State Path does not match.");
    }

    @Test
    void testGetRootDirectory() {
        File rootDirectory = Paths.get("src/test/resources").toFile();

        when(plugin.getRootDirectory()).thenReturn(rootDirectory);

        assertEquals(rootDirectory, configuration.getRootDirectory(), "Root Directory does not match.");
    }
}
