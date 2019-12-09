/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.lc;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;

import java.nio.file.Path;
import java.nio.file.Paths;

@Singleton
public class LCGeneratorPhase implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final LanguageClientConfiguration configuration;
    protected final LanguageSymbolTable symbolTable;
    protected final LCExtractor extractor;

    @Inject
    protected LCGeneratorPhase(NotificationService notifications, LanguageClientConfiguration configuration,
                               LanguageSymbolTable symbolTable, LCExtractor extractor) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.symbolTable = symbolTable;
        this.extractor = extractor;
    }

    @Override
    public String getLabel() {
        return "Language Client - Language Description";
    }

    @Override
    public int getPriority() {
        return 47;
    }

    @Override
    public void generate(GeneratorEngine engine) {
        String grammarName = this.configuration.getGrammarName().toLowerCase();
        Path outputPath = Paths.get(String.format("node/%s-templates-contribution.ts", grammarName));
        String templatePath = "templates/language-client/theia/src/node/language-templates-contribution.ftl";

        this.symbolTable.getRootNode().ifPresent(ast -> {
            this.notifications.info("Generating '%s'.", outputPath);
            engine.generate(templatePath, outputPath, ast, this.extractor);
        });
    }
}
