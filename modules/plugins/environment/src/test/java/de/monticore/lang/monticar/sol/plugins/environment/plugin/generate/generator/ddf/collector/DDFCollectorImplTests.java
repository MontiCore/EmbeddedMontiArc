/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.collector;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.configuration.EnvironmentGenerateConfiguration;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class DDFCollectorImplTests {
    @Mock NotificationService notifications;
    @Mock EnvironmentGenerateConfiguration configuration;

    @InjectMocks DDFCollectorImpl collector;

    @Test
    void testCollect() {
        File rootDirectory = new File("src/test/resources/DDFCollectorImpl");

        when(configuration.getRootDirectory()).thenReturn(rootDirectory);

        List<ASTInstruction> instructions = collector.collect();

        assertEquals(9, instructions.size(), "There should be exactly 9 instructions.");
    }
}
