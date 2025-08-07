/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.sanitizer;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class ECSanitizerImplTests {
    @Mock NotificationService notifications;
    @Mock
    ECSanitizerPhase phase;

    ECSanitizerImpl sanitizer;
    Set<ECSanitizerPhase> phases;

    @BeforeEach
    void before() {
        phases = new HashSet<>();

        phases.add(phase);

        sanitizer = new ECSanitizerImpl(notifications, phases);
    }

    @Test
    void testSanitize() {
        List<ASTInstruction> instructions = new ArrayList<>();

        assertEquals(instructions, sanitizer.sanitize(instructions), "List should be unchanged.");

        verify(phase).sanitize(instructions);
    }
}
