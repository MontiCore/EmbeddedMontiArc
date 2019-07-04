/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.template.services;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.hc.HandCodeService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template.AbstractTemplateContribution;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration;

import java.nio.file.Path;
import java.nio.file.Paths;

@Singleton
public class LanguageDiagnosticsServiceTemplate extends AbstractTemplateContribution {
    protected final LanguageServerConfiguration configuration;

    @Inject
    protected LanguageDiagnosticsServiceTemplate(LanguageServerConfiguration configuration,
                                                 HandCodeService handCode) {
        super(handCode);

        this.configuration = configuration;
    }

    @Override
    public String getTemplatePath() {
        return "templates/language-server/services/LanguageDiagnosticsService.ftl";
    }

    @Override
    public Path getOutputPath() {
        String packageStructure = this.configuration.getPackageStructure();
        String grammarName = this.configuration.getGrammarName();

        return Paths.get(String.format("%s/services/%sDiagnosticsService.java", packageStructure, grammarName));
    }

    @Override
    public Path getTopPatternOutputFile() {
        String packageStructure = this.configuration.getPackageStructure();
        String grammarName = this.configuration.getGrammarName();

        return Paths.get(String.format("%s/services/%sDiagnosticsServiceTop.java", packageStructure, grammarName));
    }
}
