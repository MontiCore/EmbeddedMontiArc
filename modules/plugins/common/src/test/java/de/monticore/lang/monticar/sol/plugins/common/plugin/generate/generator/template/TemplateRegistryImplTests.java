/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.hc.HandCodeRegistry;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable.TemplateVariableService;
import org.apache.maven.plugin.Mojo;
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
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
public class TemplateRegistryImplTests {
    final String templateRoot = "templates";
    final String templatePath = "template.ftl";
    final String outputPath = "target/Generated.java";
    final Set<TemplateContribution> contributions = new HashSet<>();

    @Mock NotificationService notifications;
    @Mock TemplateFactory factory;
    @Mock TemplateContribution contribution;
    @Mock Mojo plugin;
    @Mock HandCodeRegistry hcRegistry;
    @Mock TemplateVariableService resolver;

    Template template;
    TemplateRegistryImpl registry;

    @BeforeEach
    void before() {
        template = new TemplateImpl(templatePath, outputPath, new Object[0], resolver, hcRegistry);

        registry = new TemplateRegistryImpl(notifications, factory, contributions);

        contributions.add(contribution);
    }

    @AfterEach
    void after() {
        contributions.clear();
    }

    @Test
    void testGetTemplates() {
        when(factory.create(String.format("%s/%s", templateRoot, templatePath), outputPath, "")).thenReturn(template);

        registry.setTemplateRoot(templateRoot);
        registry.registerTemplate(templatePath, outputPath);

        assertTrue(registry.getTemplates().contains(template), "Template has not been registered.");
    }

    @Test
    void testRegisterTemplate() {
        when(factory.create(String.format("%s/%s", templateRoot, templatePath), outputPath, "")).thenReturn(template);

        registry.setTemplateRoot(templateRoot);
        registry.registerTemplate(templatePath, outputPath);

        assertEquals(1, registry.getTemplates().size(), "There should be exactly one registered template.");
        assertTrue(registry.getTemplates().contains(template), "Template has not been registered.");
    }

    @Test
    void testSetTemplateRoot() {
        registry.setTemplateRoot(templateRoot + "/");

        assertEquals(templateRoot, registry.getTemplateRoot(), "Template Root should not have a trailing slash.");

        registry.setTemplateRoot(templateRoot);

        assertEquals(templateRoot, registry.getTemplateRoot(), "Template Root does not match.");
    }

    @Test
    void testGetTemplateRoot() {
        registry.setTemplateRoot(templateRoot);

        assertEquals(templateRoot, registry.getTemplateRoot(), "Template Root does not match.");
    }

    @Test
    void testSetTopPatternSuffix() {
        registry.setTopPatternSuffix("Top");

        assertEquals("Top", registry.getTopPatternSuffix(), "Top Pattern Suffix does not match.");
    }

    @Test
    void testGetTopPatternSuffix() {
        registry.setTopPatternSuffix("Top");

        assertEquals("Top", registry.getTopPatternSuffix(), "Top Pattern Suffix does not match.");
    }

    @Test
    void testGetPriority() {
        assertEquals(60, registry.getPriority(), "Unexpected Priority.");
    }

    @Test
    void testOnPluginConfigure() {
        doAnswer(invocation -> {
            registry.registerTemplate(templatePath, outputPath);
            return invocation;
        }).when(contribution).registerTemplates(registry);

        when(factory.create(String.format("%s/%s", templateRoot, templatePath), outputPath, "")).thenReturn(template);

        registry.setTemplateRoot(templateRoot);
        registry.onPluginConfigure(plugin);

        assertEquals(1, registry.getTemplates().size(), "There should be exactly one registered template.");
        assertTrue(registry.getTemplates().contains(template), "Template has not been registered.");
    }
}
