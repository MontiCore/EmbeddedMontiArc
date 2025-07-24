/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.generator;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
public class LanguageServerTemplatesTests {
    LanguageServerTemplates templates = new LanguageServerTemplates();

    @Test
    void testRegisterTemplates() {
        TemplateRegistry registry = mock(TemplateRegistry.class);

        templates.registerTemplates(registry);

        verify(registry).setTemplateRoot("templates/language-server");
        verify(registry).setTopPatternSuffix("TOP");

        verify(registry).registerTemplate(
                "ls/LanguageServerLauncher.ftl",
                "${package}/ls/${grammarName}ServerLauncher.java"
        );

        verify(registry).registerTemplate(
                "services/LanguageDiagnosticsService.ftl",
                "${package}/services/${grammarName}DiagnosticsService.java"
        );

        verify(registry).registerTemplate(
                "LanguageModule.ftl",
                "${package}/${grammarName}Module.java"
        );
    }
}
