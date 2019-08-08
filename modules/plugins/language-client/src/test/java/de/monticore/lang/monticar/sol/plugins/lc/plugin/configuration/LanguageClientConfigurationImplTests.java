/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration;

import de.monticore.lang.monticar.sol.plugins.lc.plugin.LanguageClientPlugin;
import org.apache.maven.model.Build;
import org.apache.maven.project.MavenProject;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertIterableEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class LanguageClientConfigurationImplTests {
    @Mock LanguageClientPlugin plugin;

    LanguageClientConfigurationImpl configuration;
    File expectedFile = Paths.get("src/test/resources/LanguageClientConfigurationImpl/EmbeddedMontiArcMathAntlr.tokens").toFile();
    File resourcesPath = Paths.get("src/test/resources/LanguageClientConfigurationImpl").toFile();

    @BeforeEach
    void before() {
        MavenProject parent = mock(MavenProject.class);
        MavenProject language = mock(MavenProject.class);
        MavenProject server = mock(MavenProject.class);
        MavenProject client = mock(MavenProject.class);
        Build build = mock(Build.class);

        when(plugin.getGrammar()).thenReturn("de.monticore.lang.monticar.embeddedmontiarc.EmbeddedMontiArcMath");
        when(plugin.getGrammarModule()).thenReturn("language");
        when(plugin.getServerArtifact()).thenReturn("server:EmbeddedMontiArcMathAntlr.tokens");
        when(plugin.getMavenProject()).thenReturn(client);
        when(plugin.getModelsPath()).thenReturn(resourcesPath.getAbsoluteFile());
        when(plugin.getOutputPath()).thenReturn(resourcesPath);

        when(client.getParent()).thenReturn(parent);
        when(parent.getCollectedProjects()).thenReturn(Arrays.asList(language, server, client));
        when(language.getArtifactId()).thenReturn("language");
        when(server.getArtifactId()).thenReturn("server");
        when(client.getArtifactId()).thenReturn("client");
        when(server.getBuild()).thenReturn(build);
        when(language.getBuild()).thenReturn(build);
        when(build.getDirectory()).thenReturn(resourcesPath.getPath());
        when(client.getBasedir()).thenReturn(resourcesPath);

        configuration = new LanguageClientConfigurationImpl(plugin);
    }

    @Test
    void testGetTokensArtifact() throws Exception {
        File calculatedFile = configuration.getTokensArtifact();

        assertEquals(expectedFile, calculatedFile, ".tokens files do not match.");
    }

    @Test
    void testGetServerArtifact() throws Exception {
        File calculatedFile = configuration.getServerArtifact();

        assertEquals(expectedFile, calculatedFile, "Server Artifacts do not match.");
    }

    @Test
    void testGetModels() {
        List<File> calculatedModels = configuration.getModels();
        File expectedModel = new File(resourcesPath, "EmbeddedMontiArcMath.ld").getAbsoluteFile();
        List<File> expectedModels = Collections.singletonList(expectedModel);

        assertIterableEquals(expectedModels, calculatedModels, "Models do not match.");
    }
}
