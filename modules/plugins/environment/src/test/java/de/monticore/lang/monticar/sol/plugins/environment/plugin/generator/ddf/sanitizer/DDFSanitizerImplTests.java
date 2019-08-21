/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ddf.sanitizer;

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
public class DDFSanitizerImplTests {
    @Mock NotificationService notifications;
    @Mock DDFSanitizerPhase phase;

    DDFSanitizerImpl sanitizer;
    Set<DDFSanitizerPhase> phases;

    @BeforeEach
    void before() {
        phases = new HashSet<>();

        phases.add(phase);

        sanitizer = new DDFSanitizerImpl(notifications, phases);
    }

    @Test
    void testSanitize() {
        List<ASTInstruction> instructions = new ArrayList<>();

        assertEquals(instructions, sanitizer.sanitize(instructions), "List should be unchanged.");

        verify(phase).sanitize(instructions);
    }
}
