/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.textmate;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;
import org.json.JSONArray;
import org.json.JSONObject;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

@Singleton
public class TextMateGeneratorPhase implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final LanguageClientConfiguration configuration;
    protected final LanguageSymbolTable symbolTable;

    @Inject
    protected TextMateGeneratorPhase(NotificationService notifications, LanguageClientConfiguration configuration,
                                     LanguageSymbolTable symbolTable) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.symbolTable = symbolTable;
    }

    @Override
    public String getLabel() {
        return "Language Client - TextMate Grammar Generation";
    }

    @Override
    public int getPriority() {
        return 49;
    }

    @Override
    public void generate(GeneratorEngine engine) throws Exception {
        LanguageSymbol rootSymbol = this.symbolTable.getRootSymbol()
                .orElseThrow(() -> new Exception("Could not resolve root symbol."));
        List<String> keywords = rootSymbol.getEffectiveKeywords();
        String patterns = this.computePatterns(keywords);
        String repository = this.computeRepository(rootSymbol, keywords);
        String grammarName = this.configuration.getGrammarName().toLowerCase();
        Path outputPath = Paths.get(String.format("../data-gen/%s.tmLanguage.json", grammarName));
        String templateFile = "templates/language-client/theia/data/language-tmLanguage.ftl";

        this.notifications.info("Generating TextMate Grammar.");
        engine.generateNoA(templateFile, outputPath, patterns, repository);
    }

    protected String computePatterns(List<String> keywords) {
        JSONArray patterns = new JSONArray();

        for (String keyword : keywords) {
            JSONObject pattern = new JSONObject();

            pattern.put("include", String.format("#%s", keyword));
            patterns.put(pattern);
        }

        String value = patterns.toString(2);
        int length = value.length();

        return value.substring(1, length - 1);
    }

    protected String computeRepository(LanguageSymbol rootSymbol, List<String> keywords) throws Exception {
        JSONObject repository = new JSONObject();
        String extension = rootSymbol.getExtension()
                .orElseThrow(() -> new Exception("No extension specified in the model."));

        for (String keyword : keywords) {
            JSONObject match = new JSONObject();
            String regex = String.format("\\b%s\\b", this.escapeKeyword(keyword));
            String name = String.format("keyword.other.%s%s", keyword, extension);

            match.put("match", regex);
            match.put("name", name);

            repository.put(keyword, match);
        }

        String value = repository.toString(2);
        int length = value.length();

        return value.substring(1, length - 1);
    }

    protected String escapeKeyword(String keyword) {
        return keyword
                .replaceAll("\\(", "\\\\(")
                .replaceAll("\\)", "\\\\)");
    }
}
