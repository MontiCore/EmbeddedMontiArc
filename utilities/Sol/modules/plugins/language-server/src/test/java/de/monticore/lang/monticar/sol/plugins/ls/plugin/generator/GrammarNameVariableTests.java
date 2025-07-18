/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.generator;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
public class GrammarNameVariableTests {
    @Mock LanguageServerConfiguration configuration;

    @InjectMocks GrammarNameVariable variable;

    @Test
    void testGetIdentifier() {
        assertEquals("grammarName", variable.getIdentifier(), "Identifier should be 'grammarName'.");
    }

    @Test
    void testResolve() {
        Template template = mock(Template.class);

        when(configuration.getGrammarName()).thenReturn("EmbeddedMontiArcMath");

        assertEquals("EmbeddedMontiArcMath", variable.resolve(template));
    }
}
