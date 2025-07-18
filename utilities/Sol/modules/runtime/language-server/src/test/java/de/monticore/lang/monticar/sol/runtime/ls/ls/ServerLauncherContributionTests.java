/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.ls;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.mockito.Mockito.doCallRealMethod;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
public class ServerLauncherContributionTests {
    final String[] arguments = new String[0];

    @Mock ServerLauncherContribution contribution;

    @Test
    void testOnInitialize() throws Exception {
        doCallRealMethod().when(contribution).onInitialize(arguments);

        contribution.onInitialize(arguments);

        verify(contribution).onInitialize(arguments);
    }

    @Test
    void testOnConfigure() throws Exception {
        doCallRealMethod().when(contribution).onConfigure(arguments);

        contribution.onConfigure(arguments);

        verify(contribution).onConfigure(arguments);
    }
}
