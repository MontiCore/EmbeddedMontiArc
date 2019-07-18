/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator;

import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.collector.DDFCollector;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.partitioner.DDFPartitioner;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.sanitizer.DDFSanitizer;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.translator.DDFTranslator;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class DockerfileGeneratorTests {
    @Mock NotificationService notifications;
    @Mock DDFCollector collector;
    @Mock DDFSanitizer sanitizer;
    @Mock DDFPartitioner partitioner;
    @Mock DDFTranslator translator;

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
    void testGenerate() {
        GeneratorEngine engine = mock(GeneratorEngine.class);
        List<List<ASTInstruction>> partitions = new ArrayList<>();

        when(translator.translate(partitions, engine)).thenReturn("");

        generator.generate(engine);

        verify(engine).generateNoA("templates/Dockerfile.ftl", Paths.get("Dockerfile"), "");
    }
}
