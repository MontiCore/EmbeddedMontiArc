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
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class LDGeneratorPhaseTests {
    @Mock NotificationService notifications;
    @Mock LanguageClientConfiguration configuration;
    @Mock LDExtractor extractor;
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
    void testGenerate() throws Exception { // TODO: Write better test.
        GeneratorEngine engine = mock(GeneratorEngine.class);
        File model = Paths.get("src/test/resources/LDGeneratorPhase/EmbeddedMontiArcMath.ld").toFile();

        when(configuration.getGrammarName()).thenReturn("EmbeddedMontiArcMath");
        when(configuration.getModels()).thenReturn(Collections.singletonList(model));
        doNothing().when(notifications).info(anyString(), any());

        phase.generate(engine);
        verify(engine).generate(anyString(), any(Path.class), any(ASTLanguageCompilationUnit.class), any(LDExtractor.class));
    }
}
