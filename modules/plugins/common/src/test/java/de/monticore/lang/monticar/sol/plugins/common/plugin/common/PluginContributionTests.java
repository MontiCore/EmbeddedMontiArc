/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common;

import org.apache.maven.plugin.Mojo;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.mockito.Mockito.doCallRealMethod;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
public class PluginContributionTests {
    @Mock PluginContribution contribution;
    @Mock Mojo plugin;

    @Test
    void testOnPluginConfigure() throws Exception {
        doCallRealMethod().when(contribution).onPluginConfigure(plugin);

        contribution.onPluginConfigure(plugin);

        verify(contribution).onPluginConfigure(plugin);
    }

    @Test
    void testOnPluginExecute() throws Exception {
        doCallRealMethod().when(contribution).onPluginExecute(plugin);

        contribution.onPluginExecute(plugin);

        verify(contribution).onPluginExecute(plugin);
    }

    @Test
    void testOnPluginShutdown() throws Exception {
        doCallRealMethod().when(contribution).onPluginShutdown(plugin);

        contribution.onPluginShutdown(plugin);

        verify(contribution).onPluginShutdown(plugin);
    }
}
