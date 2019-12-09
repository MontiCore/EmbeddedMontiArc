/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator;

import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration.EnvironmentGenerateConfiguration;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.collector.ECCollector;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.partitioner.ECPartitioner;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.sanitizer.ECSanitizer;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.translator.ECTranslator;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class DockerfileGeneratorTests {
    @Mock NotificationService notifications;
    @Mock EnvironmentGenerateConfiguration configuration;
    @Mock
    ECCollector collector;
    @Mock
    ECSanitizer sanitizer;
    @Mock
    ECPartitioner partitioner;
    @Mock
    ECTranslator translator;

    @InjectMocks DockerfileGenerator generator;

    @Test
    void testGetLabel() {
        assertEquals("Environment Generator - Dockerfile Generation", generator.getLabel(), "Label does not match.");
    }

    @Test
    void testGetPriority() {
        assertEquals(100, generator.getPriority(), "Priorities do not match.");
    }

    @Test
    void testGenerate() throws IOException {
        GeneratorEngine engine = mock(GeneratorEngine.class);
        List<List<ASTInstruction>> partitions = new ArrayList<>();
        File outputPath = Paths.get("target/test-classes/DockerfileGenerator").toFile();
        File targetArtifact = new File(outputPath, "Dockerfile");

        when(configuration.getOutputPath()).thenReturn(outputPath);

        generator.generate(engine);

        assertTrue(targetArtifact.exists() && targetArtifact.delete(), "Dockerfile has not been created.");
    }
}
