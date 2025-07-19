/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.doCallRealMethod;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
public class GeneratorPhaseTests {
    @Mock GeneratorPhase phase;

    @Test
    void testConfigure() throws Exception {
        doCallRealMethod().when(phase).configure();

        phase.configure();

        verify(phase).configure();
    }

    @Test
    void testShouldGenerate() {
        doCallRealMethod().when(phase).shouldGenerate();

        assertTrue(phase.shouldGenerate(), "Phase should be executed by default.");
    }

    @Test
    void testShutdown() throws Exception {
        doCallRealMethod().when(phase).shutdown();

        phase.shutdown();

        verify(phase).shutdown();
    }
}
