/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.HashSet;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
public class TemplateVariableServiceImplTests {
    Set<TemplateVariable> variables = new HashSet<>();

    @Mock NotificationService notifications;
    @Mock TemplateVariable variable;
    @Mock Template template;

    TemplateVariableServiceImpl service;

    @BeforeEach
    void before() {
        service = new TemplateVariableServiceImpl(notifications, variables);

        when(variable.getIdentifier()).thenReturn("greeting");
        when(variable.resolve(template)).thenReturn("Hello World!");

        variables.add(variable);
    }

    @AfterEach
    void after() {
        variables.clear();
    }

    @Test
    void testResolve() {
        assertEquals("Hello World! says Paul.", service.resolve("${greeting} says Paul.", template), "Variable has not been correctly resolved.");
    }
}
