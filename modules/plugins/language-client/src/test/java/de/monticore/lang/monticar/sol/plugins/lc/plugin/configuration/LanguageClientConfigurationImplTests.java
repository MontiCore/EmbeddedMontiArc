/* (c) https://github.com/MontiCore/monticore */
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
    List<String> excludedKeywords = Arrays.asList("oo", "script");

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
        when(plugin.getOutputPath()).thenReturn(resourcesPath);
        when(plugin.getRootModel()).thenReturn("de.monticore.lang.monticar.sol.tests.ld.Root");

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
    void testGetRootModel() {
        assertEquals("de.monticore.lang.monticar.sol.tests.ld.Root", configuration.getRootModel(), "Root Models do not match.");
    }

    @Test
    void testGetFileExtension() {
        when(plugin.getExtension()).thenReturn("emam");

        assertEquals("emam", configuration.getFileExtension(), "File Extensions do not match.");

        when(plugin.getExtension()).thenReturn(".emam");

        assertEquals("emam", configuration.getFileExtension(), "File Extensions do not match.");
    }

    @Test
    void testGetExcludedKeywords() {
        when(plugin.getExcludedKeywords()).thenReturn(excludedKeywords);

        assertIterableEquals(excludedKeywords, configuration.getExcludedKeywords(), "Excluded Keywords do not match.");
    }
}
