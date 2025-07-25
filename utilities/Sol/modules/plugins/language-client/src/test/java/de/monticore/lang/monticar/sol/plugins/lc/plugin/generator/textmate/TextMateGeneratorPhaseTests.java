/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.textmate;

import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;
import org.json.JSONArray;
import org.json.JSONObject;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class TextMateGeneratorPhaseTests {
    @Mock NotificationService notifications;
    @Mock LanguageClientConfiguration configuration;
    @Mock LanguageSymbolTable symbolTable;

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
        LanguageSymbol rootSymbol = mock(LanguageSymbol.class);

        when(symbolTable.getRootSymbol()).thenReturn(Optional.of(rootSymbol));
        when(rootSymbol.getEffectiveKeywords()).thenReturn(Arrays.asList("A", "B", "C", "D"));
        when(rootSymbol.getExtension()).thenReturn(Optional.of(".test"));
        when(configuration.getGrammarName()).thenReturn("Test");

        phase.generate(engine);

        verify(engine).generateNoA(anyString(), any(Path.class), anyString(), anyString());
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
    void testComputeRepository() throws Exception {
        LanguageSymbol rootSymbol = mock(LanguageSymbol.class);
        List<String> keywords = Arrays.asList("null", "true");
        JSONObject repository = new JSONObject();
        JSONObject nullMatch = new JSONObject();
        JSONObject trueMatch = new JSONObject();

        nullMatch.put("match", "\\bnull\\b").put("name", "keyword.other.null.emam");
        trueMatch.put("match", "\\btrue\\b").put("name", "keyword.other.true.emam");
        repository.put("null", nullMatch).put("true", trueMatch);

        when(rootSymbol.getExtension()).thenReturn(Optional.of(".emam"));

        String actualRepository = phase.computeRepository(rootSymbol, keywords);
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
