/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.template;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.hc.HandCodeService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template.AbstractTemplateContribution;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration;

import java.nio.file.Path;
import java.nio.file.Paths;

@Singleton
public class LanguageModuleTemplate extends AbstractTemplateContribution {
    protected final LanguageServerConfiguration configuration;

    @Inject
    protected LanguageModuleTemplate(LanguageServerConfiguration configuration, HandCodeService handCode) {
        super(handCode);

        this.configuration = configuration;
    }

    @Override
    public String getTemplatePath() {
        return "templates/language-server/LanguageModule.ftl";
    }

    @Override
    public Path getOutputPath() {
        String packageStructure = this.configuration.getPackageStructure();
        String grammarName = this.configuration.getGrammarName();

        return Paths.get(String.format("%s/%sModule.java", packageStructure, grammarName));
    }

    @Override
    public Path getTopPatternOutputFile() {
        String packageStructure = this.configuration.getPackageStructure();
        String grammarName = this.configuration.getGrammarName();

        return Paths.get(String.format("%s/%sModuleTop.java", packageStructure, grammarName));
    }
}
