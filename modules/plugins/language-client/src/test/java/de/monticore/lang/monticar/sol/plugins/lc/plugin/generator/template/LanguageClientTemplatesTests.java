/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.template;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
public class LanguageClientTemplatesTests {
    LanguageClientTemplates templates = new LanguageClientTemplates();

    @Test
    void testRegisterTemplates() {
        TemplateRegistry registry = mock(TemplateRegistry.class);

        templates.registerTemplates(registry);

        verify(registry).setTemplateRoot("templates/language-client/theia/src");
        verify(registry).setTopPatternSuffix("-top");

        verify(registry).registerTemplate(
                "browser/language-client-contribution.ftl",
                "browser/${grammarName}-client-contribution.ts"
        );

        verify(registry).registerTemplate(
                "browser/language-frontend-module.ftl",
                "browser/${grammarName}-frontend-module.ts"
        );

        verify(registry).registerTemplate(
                "browser/language-grammar-contribution.ftl",
                "browser/${grammarName}-grammar-contribution.ts"
        );

        verify(registry).registerTemplate(
                "common/index.ftl",
                "common/index.ts"
        );

        verify(registry).registerTemplate(
                "node/language-backend-module.ftl",
                "node/${grammarName}-backend-module.ts"
        );

        verify(registry).registerTemplate(
                "node/language-server-contribution.ftl",
                "node/${grammarName}-server-contribution.ts"
        );
    }
}
