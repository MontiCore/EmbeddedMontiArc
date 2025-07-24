/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.options;

import org.apache.commons.cli.Option;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.HashSet;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.doAnswer;

@ExtendWith(MockitoExtension.class)
public class DefaultOptionsRegistryTests {
    final Set<OptionsContribution> contributions = new HashSet<>();

    @Mock OptionsContribution contribution;

    DefaultOptionsRegistry registry = new DefaultOptionsRegistry(contributions);

    @BeforeEach
    void before() {
        contributions.add(contribution);
    }

    @AfterEach
    void after() {
        contributions.clear();
    }

    @Test
    void testGetOptions() {
        Option option = new Option("option", "This is an option.");

        registry.registerOption(option);

        assertEquals(1, registry.getOptions().getOptions().size(), "There should be exactly one option");
        assertTrue(registry.getOptions().getOptions().contains(option), "Option has not correctly been added.");
    }

    @Test
    void testRegisterOption() {
        Option option = new Option("option", "This is an option.");

        registry.registerOption(option);

        assertEquals(1, registry.getOptions().getOptions().size(), "There should be exactly one option");
        assertTrue(registry.getOptions().getOptions().contains(option), "Option has not correctly been added.");
    }

    @Test
    void testOnInitialize() {
        Option option = new Option("option", "This is an option.");

        doAnswer(invocation -> {
            registry.registerOption(option);
            return invocation;
        }).when(contribution).registerOptions(registry);

        registry.onInitialize(new String[] {});

        assertEquals(1, registry.getOptions().getOptions().size(), "There should be exactly one option");
        assertTrue(registry.getOptions().getOptions().contains(option), "Option has not correctly been added.");
    }
}
