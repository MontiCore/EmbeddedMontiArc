/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.ls;

import de.monticore.lang.monticar.sol.runtime.ls.options.OptionsRegistry;
import org.apache.commons.cli.Option;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
public class PortOptionTests {
    @Mock OptionsRegistry registry;

    PortOption option = new PortOption();

    @Test
    void testRegisterOptions() {
        option.registerOptions(registry);

        verify(registry).registerOption(any(Option.class));
    }
}
