/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.ld;

import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.nio.file.Path;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class LDGeneratorPhaseTests {
    @Mock NotificationService notifications;
    @Mock LanguageClientConfiguration configuration;
    @Mock LDExtractor extractor;
    @Mock LanguageSymbolTable symbolTable;

    @InjectMocks LDGeneratorPhase phase;

    @Test
    void testGetLabel() {
        assertNotNull(phase.getLabel(), "Label has not been set.");
    }

    @Test
    void testGetPriority() {
        assertTrue(phase.getPriority() < 500, "Priority should be less than the one of NotificationServiceImpl.");
    }

    @Test
    void testGenerate() { // TODO: Write better test.
        GeneratorEngine engine = mock(GeneratorEngine.class);
        ASTLanguageCompilationUnit node = mock(ASTLanguageCompilationUnit.class);

        when(configuration.getGrammarName()).thenReturn("EmbeddedMontiArc");
        when(symbolTable.getRootNode()).thenReturn(Optional.of(node));

        phase.generate(engine);
        verify(engine).generate(anyString(), any(Path.class), any(ASTLanguageCompilationUnit.class), any(LDExtractor.class));
    }
}
