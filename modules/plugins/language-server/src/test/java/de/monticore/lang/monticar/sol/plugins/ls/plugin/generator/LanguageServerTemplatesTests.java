/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.generator;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.generator.LanguageServerTemplates;
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
        verify(registry).setTopPatternSuffix("Top");

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
