/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.template;

import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;

@Singleton
public class LanguageClientTemplates implements TemplateContribution {
    @Override
    public void registerTemplates(TemplateRegistry registry) {
        registry.setTemplateRoot("templates/language-client/theia/src");
        registry.setTopPatternSuffix("-top");

        registry.registerTemplate(
                "browser/language-client-contribution.ftl",
                "browser/${grammarName}-client-contribution.ts"
        );

        registry.registerTemplate(
                "browser/language-frontend-module.ftl",
                "browser/${grammarName}-frontend-module.ts"
        );

        registry.registerTemplate(
                "browser/language-grammar-contribution.ftl",
                "browser/${grammarName}-grammar-contribution.ts"
        );

        registry.registerTemplate(
                "common/index.ftl",
                "common/index.ts"
        );

        registry.registerTemplate(
                "node/language-backend-module.ftl",
                "node/${grammarName}-backend-module.ts"
        );

        registry.registerTemplate(
                "node/language-server-contribution.ftl",
                "node/${grammarName}-server-contribution.ts"
        );
    }
}
