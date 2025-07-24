/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.template;

import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateDeclarationSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.ValidatorPrinter;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.util.Collections;
import java.util.Optional;

import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class LanguageClientTemplatesTests {
    @Mock LanguageSymbolTable symbolTable;
    @Mock ValidatorPrinter printer;

    @InjectMocks LanguageClientTemplates templates;

    @Test
    void testRegisterTemplates() {
        TemplateRegistry registry = mock(TemplateRegistry.class);
        LanguageSymbol rootSymbol = mock(LanguageSymbol.class);
        TemplateDeclarationSymbol declaration = mock(TemplateDeclarationSymbol.class);

        when(symbolTable.getRootSymbol()).thenReturn(Optional.of(rootSymbol));
        when(rootSymbol.getLocalDeclarationSymbols()).thenReturn(Collections.singletonList(declaration));
        when(declaration.getName()).thenReturn("Some Name");

        templates.registerTemplates(registry);

        verify(registry, times(2)).setTemplateRoot("templates/language-client/theia/src");
        verify(registry).setTopPatternSuffix("-top");

        verify(registry).registerTemplate(
                "browser/language-client-contribution.ftl",
                "browser/${grammarName}-client-contribution.ts"
        );

        verify(registry).registerTemplate(
                "browser/language-frontend-module.ftl",
                "browser/${grammarName}-frontend-module.ts",
                rootSymbol
        );

        verify(registry).registerTemplate(
                "browser/language-grammar-contribution.ftl",
                "browser/${grammarName}-grammar-contribution.ts"
        );

        verify(registry).registerTemplate(
                "common/language-protocol.ftl",
                "common/${grammarName}-protocol.ts"
        );

        verify(registry).registerTemplate(
                "node/language-backend-module.ftl",
                "node/${grammarName}-backend-module.ts",
                rootSymbol
        );

        verify(registry).registerTemplate(
                "node/language-server-contribution.ftl",
                "node/${grammarName}-server-contribution.ts"
        );
    }
}
