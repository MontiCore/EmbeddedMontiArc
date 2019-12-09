/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.Set;

import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
public class AbstractTemplateGeneratorPhaseTests {
    @Mock NotificationService notifications;
    @Mock TemplateRegistry registry;
    @Mock GeneratorEngine engine;
    @Mock Template template;

    AbstractTemplateGeneratorPhase phase;

    @BeforeEach
    void before() {
        phase = new AbstractTemplateGeneratorPhase(notifications, registry) {
            @Override
            public String getLabel() {
                return null;
            }

            @Override
            public int getPriority() {
                return 0;
            }
        };
    }

    @Test
    public void testGenerate() {
        Set<Template> templates = new HashSet<>();
        String templatePath = "Template.ftl";
        Path outputPath = Paths.get("Out.java");
        Path outputPathTop = Paths.get("OutTop.java");
        Object[] arguments = new Object[0];

        templates.add(template);

        when(registry.getTemplates()).thenReturn(templates);
        when(template.getTemplatePath()).thenReturn(templatePath);
        when(template.getOutputPath()).thenReturn(outputPath);
        when(template.hasHandwrittenPeer()).thenReturn(false);
        when(template.getArguments()).thenReturn(arguments);

        phase.generate(engine);

        verify(engine).generateNoA(templatePath, outputPath, arguments);

        when(template.hasHandwrittenPeer()).thenReturn(true);
        when(template.getTopPatternOutputPath()).thenReturn(outputPathTop);

        phase.generate(engine);

        verify(engine).generateNoA(templatePath, outputPathTop, arguments);
    }
}
