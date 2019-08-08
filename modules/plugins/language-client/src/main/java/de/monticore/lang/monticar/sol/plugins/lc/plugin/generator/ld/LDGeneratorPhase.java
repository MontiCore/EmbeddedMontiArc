/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.ld;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.language._parser.LanguageParser;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

@Singleton
public class LDGeneratorPhase implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final LanguageClientConfiguration configuration;
    protected final LDExtractor extractor;

    @Inject
    protected LDGeneratorPhase(NotificationService notifications, LanguageClientConfiguration configuration,
                               LDExtractor extractor) {
        this.notifications = notifications;
        this.configuration = configuration;
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
    public void generate(GeneratorEngine engine) throws Exception {
        String grammarName = this.configuration.getGrammarName().toLowerCase();
        List<File> models = this.configuration.getModels();
        LanguageParser parser = new LanguageParser();
        Path outputPath = Paths.get(String.format("node/%s-templates-contribution.ts", grammarName));
        String templatePath = "templates/language-client/theia/src/node/language-templates-contribution.ftl";

        this.notifications.info("Generating '%s'.", outputPath);

        for (File model : models) {
            parser.parse(model.getPath()).ifPresent(ast -> engine.generate(templatePath, outputPath, ast, this.extractor));
        }
    }
}
