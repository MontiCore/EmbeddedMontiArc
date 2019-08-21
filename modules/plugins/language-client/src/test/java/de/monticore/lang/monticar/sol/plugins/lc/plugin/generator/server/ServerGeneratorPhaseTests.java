/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.server;

import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.nio.file.Paths;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class ServerGeneratorPhaseTests {
    @Mock NotificationService notifications;
    @Mock LanguageClientConfiguration configuration;
    @Mock NPMPackageService packages;
    @InjectMocks ServerGeneratorPhase phase;

    @Test
    void testGetLabel() {
        assertNotNull(phase.getLabel(), "Label has not been set.");
    }

    @Test
    void testGetPriority() {
        assertTrue(phase.getPriority() < 500, "Priority should be less than the one of NotificationServiceImpl.");
    }

    @Test
    void testGenerate() throws Exception {
        GeneratorEngine engine = mock(GeneratorEngine.class);
        SolPackage rootPackage = mock(SolPackage.class);
        File serverArtifact = Paths.get("src/test/resources/ServerGeneratorPhase/Dummy.txt").toFile();
        File outputPath = Paths.get("target/test-classes/ServerGeneratorPhase").toFile();
        File targetArtifact = Paths.get(outputPath.getPath(), "server", "EmbeddedMontiArcMath.jar").toFile();

        when(configuration.getGrammarName()).thenReturn("EmbeddedMontiArcMath");
        when(configuration.getServerArtifact()).thenReturn(serverArtifact);
        when(configuration.getOutputPath()).thenReturn(outputPath);
        when(packages.getCurrentPackage()).thenReturn(Optional.of(rootPackage));
        when(rootPackage.getDirectory("server")).thenReturn(Optional.of("server"));
        doNothing().when(notifications).info(anyString(), any(), any());

        phase.generate(engine);

        assertTrue(targetArtifact.delete() && targetArtifact.getParentFile().delete(), "Server Artifact has not been copied.");
    }
}
