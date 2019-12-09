/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import org.apache.maven.plugin.Mojo;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
public class GeneratorImplTests {
    final List<GeneratorPhase> phases = new ArrayList<>();
    final Set<GlexContribution> glexes = new HashSet<>();
    final Set<GeneratorSetupContribution> setups = new HashSet<>();

    @Mock NotificationService notifications;
    @Mock GeneratorPhase phase;
    @Mock GlexContribution glexContribution;
    @Mock GeneratorSetupContribution setupContribution;
    @Mock Mojo plugin;
    @Mock GeneratorSetup setup;
    @Mock GeneratorEngine engine;

    GeneratorImpl generator;

    @BeforeEach
    void before() {
        phases.add(phase);
        glexes.add(glexContribution);
        setups.add(setupContribution);

        generator = new GeneratorImpl(notifications, engine, setup, phases, glexes, setups);
    }

    @AfterEach
    void after() {
        phases.clear();
        glexes.clear();
        setups.clear();
    }

    @Test
    void testGetPriority() {
        assertEquals(40, generator.getPriority(), "Unexpected Priority.");
    }

    @Test
    void testConfigure() throws Exception {
        GlobalExtensionManagement glex = mock(GlobalExtensionManagement.class);

        when(setup.getGlex()).thenReturn(glex);

        generator.configure();

        verify(setupContribution).setup(generator.setup);
        verify(glexContribution).defineGlobalVars(generator.setup.getGlex());
        verify(phase).configure();
    }

    @Test
    void testGenerate() throws Exception {
        when(phase.getLabel()).thenReturn("Dummy");
        when(phase.shouldGenerate()).thenReturn(true);

        generator.generate();

        verify(phase).shouldGenerate();
        verify(phase).generate(generator.engine);

        when(phase.shouldGenerate()).thenReturn(false);

        generator.onPluginExecute(plugin);

        verify(phase, times(1)).generate(generator.engine);
    }

    @Test
    void testShutdown() throws Exception {
        generator.shutdown();

        verify(phase).shutdown();
    }

    @Test
    void testOnPluginConfigure() throws Exception {
        GlobalExtensionManagement glex = mock(GlobalExtensionManagement.class);

        when(setup.getGlex()).thenReturn(glex);
        generator.onPluginConfigure(plugin);

        verify(setupContribution).setup(generator.setup);
        verify(glexContribution).defineGlobalVars(setup.getGlex());
        verify(phase).configure();
    }

    @Test
    void testOnPluginExecute() throws Exception {
        when(phase.getLabel()).thenReturn("Dummy");
        when(phase.shouldGenerate()).thenReturn(true);

        generator.onPluginExecute(plugin);

        verify(phase).shouldGenerate();
        verify(phase).generate(generator.engine);

        when(phase.shouldGenerate()).thenReturn(false);

        generator.onPluginExecute(plugin);

        verify(phase, times(1)).generate(generator.engine);
    }

    @Test
    void testOnPluginShutdown() throws Exception {
        generator.onPluginShutdown(plugin);

        verify(phase).shutdown();
    }
}
