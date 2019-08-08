/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.textmate;

import com.google.common.base.Charsets;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GeneratorPhase;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import org.apache.commons.io.FileUtils;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.function.Predicate;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

@Singleton
public class TextMateGeneratorPhase implements GeneratorPhase {
    protected final NotificationService notifications;
    protected final LanguageClientConfiguration configuration;

    @Inject
    protected TextMateGeneratorPhase(NotificationService notifications, LanguageClientConfiguration configuration) {
        this.notifications = notifications;
        this.configuration = configuration;
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
        Predicate<String> predicate = keyword -> !this.configuration.getExcludedKeywords().contains(keyword);
        List<String> keywords = this.computeKeywords().stream().filter(predicate).collect(Collectors.toList());
        String patterns = this.computePatterns(keywords);
        String repository = this.computeRepository(keywords);
        String grammarName = this.configuration.getGrammarName().toLowerCase();
        Path outputPath = Paths.get(String.format("../data-gen/%s.tmLanguage.json", grammarName));
        String templateFile = "templates/language-client/theia/data/language-tmLanguage.ftl";

        this.notifications.info("Generating TextMate Grammar.");
        engine.generateNoA(templateFile, outputPath, patterns, repository);
    }

    protected List<String> computeKeywords() throws IOException {
        File tokensFile = this.configuration.getTokensArtifact();
        List<String> lines = FileUtils.readLines(tokensFile, Charsets.UTF_8);
        Pattern pattern = Pattern.compile("'(\\w+[_-]?\\w*)'=\\d+");
        Stream<Matcher> filteredLines = lines.stream().map(pattern::matcher).filter(Matcher::matches);

        return filteredLines.map(matcher -> matcher.group(1)).collect(Collectors.toList());
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

    protected String computeRepository(List<String> keywords) {
        JSONObject repository = new JSONObject();

        for (String keyword : keywords) {
            JSONObject match = new JSONObject();
            String regex = String.format("\\b%s\\b", this.escapeKeyword(keyword));
            String name = String.format("keyword.other.%s.%s", keyword, this.configuration.getFileExtension());

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
