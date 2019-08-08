/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.textmate;

import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import org.json.JSONArray;
import org.json.JSONObject;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
public class TextMateGeneratorPhaseTests {
    @Mock NotificationService notifications;
    @Mock LanguageClientConfiguration configuration;

    @InjectMocks TextMateGeneratorPhase phase;

    @Test
    void testGetLabel() {
        assertNotNull(phase.getLabel(), "Label has not been set.");
    }

    @Test
    void testGetPriority() {
        assertTrue(phase.getPriority() < 500, "Priority should be less than the one of NotificationServiceImpl.");
    }

    @Test
    void testGenerate() throws Exception { // TODO: Write better test.
        GeneratorEngine engine = mock(GeneratorEngine.class);
        List<String> excludes = Arrays.asList("null", "true", "false", "void");
        File tokensFile = Paths.get("src/test/resources/TextMateGeneratorPhase/EmbeddedMontiArcMathAntlr.tokens").toFile();

        when(configuration.getTokensArtifact()).thenReturn(tokensFile);
        when(configuration.getFileExtension()).thenReturn("emam");
        when(configuration.getExcludedKeywords()).thenReturn(excludes);
        when(configuration.getGrammarName()).thenReturn("EmbeddedMontiArcMath");

        phase.generate(engine);

        verify(engine).generateNoA(anyString(), any(Path.class), anyString(), anyString());
    }

    @Test
    void testComputeKeywords() throws Exception {
        List<String> expectedKeywords = Arrays.asList("null", "true", "false", "void", "boolean");
        File tokensFile = Paths.get("src/test/resources/TextMateGeneratorPhase/EmbeddedMontiArcMathAntlr.tokens").toFile();

        when(configuration.getTokensArtifact()).thenReturn(tokensFile);

        List<String> actualKeywords = phase.computeKeywords();

        assertEquals(expectedKeywords, actualKeywords, "Keywords do not match.");
    }

    @Test
    void testComputePatterns() {
        List<String> keywords = Arrays.asList("null", "true");
        String actualPatterns = phase.computePatterns(keywords);
        JSONArray wrapper = new JSONArray();
        JSONObject nullPattern = new JSONObject();
        JSONObject truePattern = new JSONObject();

        nullPattern.put("include", "#null");
        truePattern.put("include", "#true");
        wrapper.put(nullPattern).put(truePattern);

        String expectedValue = wrapper.toString(2);
        int expectedLength = expectedValue.length();
        String expectedPatterns = expectedValue.substring(1, expectedLength - 1);

        assertEquals(expectedPatterns, actualPatterns, "Patterns do not match.");
    }

    @Test
    void testComputeRepository() {
        List<String> keywords = Arrays.asList("null", "true");
        JSONObject repository = new JSONObject();
        JSONObject nullMatch = new JSONObject();
        JSONObject trueMatch = new JSONObject();

        nullMatch.put("match", "\\bnull\\b").put("name", "keyword.other.null.emam");
        trueMatch.put("match", "\\btrue\\b").put("name", "keyword.other.true.emam");
        repository.put("null", nullMatch).put("true", trueMatch);

        when(configuration.getFileExtension()).thenReturn("emam");

        String actualRepository = phase.computeRepository(keywords);
        String expectedValue = repository.toString(2);
        int expectedLength = expectedValue.length();
        String expectedRepository = expectedValue.substring(1, expectedLength - 1);

        assertEquals(expectedRepository, actualRepository, "Repository does not match.");
    }

    @Test
    void testEscapeKeyword() {
        assertEquals("\\(", phase.escapeKeyword("("), "Keyword is not correctly escaped.");
    }
}
