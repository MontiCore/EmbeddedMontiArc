/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.generator;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.generator.PackageVariable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
public class PackageVariableTests {
    @Mock LanguageServerConfiguration configuration;

    @InjectMocks PackageVariable variable;

    @Test
    void testGetIdentifier() {
        assertEquals("package", variable.getIdentifier(), "Identifier does not match.");
    }

    @Test
    void testResolve() {
        Template template = mock(Template.class);

        when(configuration.getPackageStructure()).thenReturn("org/something/something");

        assertEquals("org/something/something", variable.resolve(template));
    }
}
