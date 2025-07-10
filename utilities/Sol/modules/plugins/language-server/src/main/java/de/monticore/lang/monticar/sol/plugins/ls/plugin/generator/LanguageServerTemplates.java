/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.generator;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;

public class LanguageServerTemplates implements TemplateContribution {
    @Override
    public void registerTemplates(TemplateRegistry registry) {
        registry.setTemplateRoot("templates/language-server");
        registry.setTopPatternSuffix("TOP");

        registry.registerTemplate(
                "ls/LanguageServerLauncher.ftl",
                "${package}/ls/${grammarName}ServerLauncher.java"
        );

        registry.registerTemplate(
                "services/LanguageDiagnosticsService.ftl",
                "${package}/services/${grammarName}DiagnosticsService.java"
        );

        registry.registerTemplate(
                "LanguageModule.ftl",
                "${package}/${grammarName}Module.java"
        );
    }
}
