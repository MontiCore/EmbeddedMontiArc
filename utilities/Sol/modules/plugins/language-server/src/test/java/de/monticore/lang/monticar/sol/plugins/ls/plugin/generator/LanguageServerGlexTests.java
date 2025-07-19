/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.generator;

import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.generator.LanguageServerGlex;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
public class LanguageServerGlexTests {
    @Mock LanguageServerConfiguration configuration;

    @InjectMocks LanguageServerGlex lsGlex;

    @Test
    void testDefineGlobalVars() {
        GlobalExtensionManagement glex = mock(GlobalExtensionManagement.class);

        lsGlex.defineGlobalVars(glex);

        verify(glex).defineGlobalVar("configuration", configuration);
    }
}
