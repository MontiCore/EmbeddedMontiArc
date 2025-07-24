/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.generator;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.generator.TemplateGeneratorPhase;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.assertEquals;

@ExtendWith(MockitoExtension.class)
public class TemplateGeneratorPhaseTests {
    @Mock NotificationService notifications;
    @Mock TemplateRegistry registry;

    @InjectMocks TemplateGeneratorPhase phase;

    @Test
    void testGetLabel() {
        assertEquals("Language Server Generator - Template Generation", phase.getLabel(), "Label does not match.");
    }

    @Test
    void testGetPriority() {
        assertEquals(50, phase.getPriority(), "Priority does not match.");
    }
}
