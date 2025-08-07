/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;

public interface LanguageClientConfiguration extends GeneratePluginConfiguration {
    /**
     * @return The qualified name of the grammar.
     */
    String getGrammarQualifiedName();

    /**
     * @return The name of the grammar.
     */
    String getGrammarName();

    /**
     * @return The root model of the language description.
     */
    String getRootModel();
}
