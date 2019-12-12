/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.template;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;

@Singleton
public class LanguageClientTemplates implements TemplateContribution {
    protected static final String FRONTEND = "Frontend";
    protected static final String BACKEND = "Backend";

    protected final LanguageSymbolTable symbolTable;

    @Inject
    protected LanguageClientTemplates(LanguageSymbolTable symbolTable) {
        this.symbolTable = symbolTable;
    }

    @Override
    public void registerTemplates(TemplateRegistry registry) {
        this.symbolTable.getRootSymbol().ifPresent(language -> {
            registry.setTemplateRoot("templates/language-client/theia/src");
            registry.setTopPatternSuffix("-top");

            registry.registerTemplate(
                "browser/language-client-contribution.ftl",
                "browser/${grammarName}-client-contribution.ts"
            );

            registry.registerTemplate(
                "browser/language-frontend-module.ftl",
                "browser/${grammarName}-frontend-module.ts",
                language
            );

            registry.registerTemplate(
                "browser/language-grammar-contribution.ftl",
                "browser/${grammarName}-grammar-contribution.ts"
            );

            registry.registerTemplate(
                "common/language-protocol.ftl",
                "common/${grammarName}-protocol.ts"
            );

            registry.registerTemplate(
                "node/language-backend-module.ftl",
                "node/${grammarName}-backend-module.ts",
                language
            );

            registry.registerTemplate(
                "node/language-server-contribution.ftl",
                "node/${grammarName}-server-contribution.ts"
            );

            registry.registerTemplate(
                "common/language-validator-contribution.ftl",
                "node/${grammarName}-backend-validator-contribution.ts",
                language, BACKEND
            );

            registry.registerTemplate(
                "common/language-validator-contribution.ftl",
                "browser/${grammarName}-frontend-validator-contribution.ts",
                language, FRONTEND
            );

            language.getLocalDeclarationSymbols().forEach(declaration -> {
                String declarationName = declaration.getName().toLowerCase();

                registry.setTemplateRoot("templates/option");

                registry.registerTemplate(
                    "common/protocol.ftl",
                    String.format("common/validators/%s-protocol.ts", declarationName),
                    declaration
                );

                registry.setTemplateRoot("templates/language-client/theia/src");

                registry.registerTemplate(
                    "common/language-end-validator.ftl",
                    String.format("browser/validators/%s-frontend-validator.ts", declarationName),
                    declaration, FRONTEND
                );

                registry.registerTemplate(
                    "common/language-end-validator.ftl",
                    String.format("node/validators/%s-backend-validator.ts", declarationName),
                    declaration, BACKEND
                );
            });
        });
    }
}
