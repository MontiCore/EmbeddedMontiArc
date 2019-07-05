/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.template.theia.browser;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.hc.HandCodeService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template.AbstractTemplateContribution;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;

import java.nio.file.Path;
import java.nio.file.Paths;

@Singleton
public class LanguageClientContributionTemplate extends AbstractTemplateContribution {
    protected final LanguageClientConfiguration configuration;

    @Inject
    protected LanguageClientContributionTemplate(LanguageClientConfiguration configuration, HandCodeService handCode) {
        super(handCode);

        this.configuration = configuration;
    }

    @Override
    public String getTemplatePath() {
        return "templates/language-client/theia/src/browser/language-client-contribution.ftl";
    }

    @Override
    public Path getOutputPath() {
        String grammarName = this.configuration.getGrammarName().toLowerCase();

        return Paths.get(String.format("browser/%s-client-contribution.ts", grammarName));
    }

    @Override
    public Path getTopPatternOutputFile() {
        String grammarName = this.configuration.getGrammarName().toLowerCase();

        return Paths.get(String.format("browser/%s-client-contribution-top.ts", grammarName));
    }
}
