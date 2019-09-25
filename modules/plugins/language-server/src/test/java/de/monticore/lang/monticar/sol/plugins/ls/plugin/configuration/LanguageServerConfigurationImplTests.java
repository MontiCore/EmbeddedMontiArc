/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration;

import de.monticore.lang.monticar.sol.plugins.ls.plugin.LanguageServerPlugin;
import org.apache.maven.project.MavenProject;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.io.File;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
public class LanguageServerConfigurationImplTests {
    final File baseDir = new File("");
    final String grammar = "de.monticore.lang.embeddedmontiarc.EmbeddedMontiArcMath";

    @Mock LanguageServerPlugin plugin;

    LanguageServerConfigurationImpl configuration;

    @BeforeEach
    void before() {
        MavenProject mavenProject = mock(MavenProject.class);

        when(plugin.getMavenProject()).thenReturn(mavenProject);
        when(plugin.getOutputPath()).thenReturn(baseDir);
        when(mavenProject.getBasedir()).thenReturn(baseDir);

        configuration = new LanguageServerConfigurationImpl(plugin);
    }

    @Test
    void testGetDefaultHandCodedPath() {
        File defaultPath = new File(baseDir, "src/main/java");

        assertEquals(defaultPath, configuration.getDefaultHandCodedPath(), "Default Handwritten Paths do not match.");
    }

    @Test
    void testGetGrammarQualifiedName() {
        when(plugin.getGrammar()).thenReturn(grammar);

        assertEquals(grammar, configuration.getGrammarQualifiedName(), "Qualified Name of Grammar does not match.");
    }

    @Test
    void testGetGrammarName() {
        when(plugin.getGrammar()).thenReturn(grammar);

        assertEquals("EmbeddedMontiArcMath", configuration.getGrammarName(), "Name of Grammar does not match.");
    }

    @Test
    void testGetGrammarGeneratedPackage() {
        when(plugin.getGrammar()).thenReturn(grammar);

        assertEquals(grammar.toLowerCase(), configuration.getGrammarGeneratedPackage(), "Generated Package of Grammar does not match.");
    }

    @Test
    void testGetSourceCodeOutputPath() {
        File statePath = new File(baseDir, "sourcecode");

        assertEquals(statePath, configuration.getSourceCodeOutputPath(), "Source Output Path does not match.");
    }

    @Test
    void testGetPackageStructure() {
        String packageStructure = grammar.toLowerCase().replaceAll("\\.", "/");

        when(plugin.getGrammar()).thenReturn(grammar);

        assertEquals(packageStructure, configuration.getPackageStructure(), "Package Structure does not match.");
    }
}
