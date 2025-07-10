/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.ls;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.HashSet;
import java.util.Set;

import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
public class AbstractServerLauncherTests {
    final String[] arguments = new String[0];
    final Set<ServerLauncherContribution> contributions = new HashSet<>();

    @Mock ServerLauncherContribution contribution;

    AbstractServerLauncher launcher = new AbstractServerLauncher(contributions) {
        @Override
        public void launch(String[] arguments) {}
    };

    @BeforeEach
    void before() {
        contributions.add(contribution);
    }

    @AfterEach
    void after() {
        contributions.clear();
    }

    @Test
    void testInitialize() throws Exception {
        launcher.initialize(arguments);

        verify(contribution).onInitialize(arguments);
    }

    @Test
    void testConfigure() throws Exception {
        launcher.configure(arguments);

        verify(contribution).onConfigure(arguments);
    }
}
