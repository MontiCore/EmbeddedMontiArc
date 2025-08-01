/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.template;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable.TemplateVariable;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;

@Singleton
public class GrammarNameVariable implements TemplateVariable {
    protected final LanguageClientConfiguration configuration;

    @Inject
    protected GrammarNameVariable(LanguageClientConfiguration configuration) {
        this.configuration = configuration;
    }

    @Override
    public String getIdentifier() {
        return "grammarName";
    }

    @Override
    public String resolve(Template template) {
        return this.configuration.getGrammarName().toLowerCase();
    }
}
