/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.hc.HandCodeRegistry;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable.TemplateVariableService;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
public class TemplateImplTests {
    String templatePath = "templates/template.ftl";
    String outputPath = "target/Generated.java";
    String suffix = "Top";

    @Mock TemplateVariableService resolver;
    @Mock HandCodeRegistry registry;

    TemplateImpl template;

    @BeforeEach
    void before() {
        template = new TemplateImpl(templatePath, outputPath, suffix, new Object[0], resolver, registry);
    }

    @Test
    void testGetTemplatePath() {
        assertEquals(templatePath, template.getTemplatePath(), "Template Paths do not match");
    }

    @Test
    void testGetOutputPath() {
        when(resolver.resolve(outputPath, template)).thenReturn(outputPath);
        assertEquals(Paths.get(outputPath), template.getOutputPath(), "Output Paths do not match.");
    }

    @Test
    void testGetTopPatternOutputPath() {
        when(resolver.resolve(outputPath, template)).thenReturn(outputPath);
        assertEquals(Paths.get("target/GeneratedTop.java"), template.getTopPatternOutputPath(), "Output Paths do not match");
    }

    @Test
    void testHasHandwrittenPeer() {
        Set<Path> handCodes = new HashSet<>();

        handCodes.add(Paths.get(outputPath));
        when(resolver.resolve(outputPath, template)).thenReturn(outputPath);
        when(registry.getHandCodes()).thenReturn(handCodes);

        assertTrue(template.hasHandwrittenPeer(), "Template should have Handwritten Peer.");

        handCodes.clear();
        assertFalse(template.hasHandwrittenPeer(), "Template should not have any Handwritten Peer.");
    }

    @Test
    void testToString() {
        when(registry.getHandCodes()).thenReturn(new HashSet<>());
        assertEquals("{ Template: templates/template.ftl, HandwrittenPeer: false }", template.toString(), "String representations do not match.");
    }

    @Test
    void testEquals() {
        TemplateImpl peer = new TemplateImpl(templatePath, outputPath, suffix, new Object[0], resolver, registry);

        when(resolver.resolve(any(String.class), any(Template.class))).thenReturn(outputPath);
        when(registry.getHandCodes()).thenReturn(new HashSet<>());

        assertEquals(template, peer, "Templates should be equal.");
    }
}
