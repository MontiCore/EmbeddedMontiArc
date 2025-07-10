/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.template;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.*;

@ExtendWith(MockitoExtension.class)
public class TemplateGeneratorPhaseTests {
    @Mock NotificationService notifications;
    @Mock TemplateRegistry registry;

    @InjectMocks TemplateGeneratorPhase phase;

    @Test
    void testGetLabel() {
        assertNotNull(phase.getLabel(), "Label has not been set.");
    }

    @Test
    void testGetPriority() {
        assertTrue(phase.getPriority() < 500, "Priority should be less than the one of NotificationServiceImpl.");
    }
}
